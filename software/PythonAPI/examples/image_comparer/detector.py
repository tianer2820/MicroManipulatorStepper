import cv2
import numpy as np
from typer import Typer
from pathlib import Path

APP = Typer()

DEFAULT_MATCH_LIMIT = 0.8
DEFAULT_RANSAC_THRESHOLD = 5
MAX_WIDTH = 1600
MAX_HEIGHT = 900
DEFAULT_PIXEL_UM = 1.0
DEFAULT_CAMERA_HEIGHT_UM = 1000.0   # height of camera above the plane (in µm)
DEFAULT_FOV_DEG = 60.0              # diagonal field of view in degrees


def build_camera_intrinsics(img_shape: tuple, fov_deg: float) -> np.ndarray:
    """
    Build a simple pinhole camera intrinsic matrix from image size and diagonal FOV.

    Args:
        img_shape: (height, width) of the image in pixels.
        fov_deg:   Diagonal field-of-view in degrees.

    Returns:
        3×3 intrinsic matrix K.
    """
    h, w = img_shape[:2]
    diag_px = np.sqrt(w**2 + h**2)
    fov_rad = np.radians(fov_deg)
    # focal length from diagonal FOV
    f = (diag_px / 2.0) / np.tan(fov_rad / 2.0)
    cx, cy = w / 2.0, h / 2.0
    K = np.array([
        [f,  0, cx],
        [0,  f, cy],
        [0,  0,  1],
    ], dtype=np.float64)
    return K


def pixels_to_world(points_px: np.ndarray, K: np.ndarray, camera_height: float) -> np.ndarray:
    """
    Back-project 2-D image points onto the Z=0 ground plane, given the camera
    is at height `camera_height` above that plane (looking straight down).

    Returns an (N, 3) array of 3-D world points with Z=0.
    """
    K_inv = np.linalg.inv(K)
    pts_h = np.hstack([points_px, np.ones((len(points_px), 1))])   # (N,3) homogeneous
    rays = (K_inv @ pts_h.T).T                                       # (N,3) normalised ray dirs
    # intersect with Z=0 plane: world_point = camera_origin + t * ray
    # camera_origin = (0, 0, camera_height), ray direction has ray_z component
    # t = -camera_height / ray[:, 2]  (ray_z is negative for a downward-looking cam)
    t = camera_height / rays[:, 2]    # works when looking *toward* +Z ground plane
    world_pts = rays * t[:, np.newaxis]
    world_pts[:, 2] = 0.0             # enforce Z=0 on the ground plane
    return world_pts.astype(np.float64)


def compare_images(
    img1_path: Path,
    img2_path: Path,
    match_limit: float,
    camera_height: float,
    fov_deg: float,
    pixel_to_um: float,
):
    """
    Detect and match SIFT features between two images, then estimate the 3-D
    transform using a two-stage approach:

      Stage 1 — estimateAffinePartial2D (RANSAC):
        Recovers X/Y translation and in-plane rotation (yaw) accurately.
        The affine scale factor is used to derive Z.

      Stage 2 — Z from scale:
        A scale factor s != 1 means camera 2 was closer/farther than camera 1.
        Given the camera was at height H above the plane:
            new_height = H / s        (larger features → smaller height → negative ΔZ)
            ΔZ = new_height − H = H * (1/s − 1)

        solvePnP is NOT used for the full pose because its tvec.Z encodes the
        distance from the world plane origin, not the *change* in camera height,
        leading to large spurious Z values for pure lateral motion.
    """
    img1 = cv2.imread(str(img1_path), cv2.IMREAD_GRAYSCALE)
    img2 = cv2.imread(str(img2_path), cv2.IMREAD_GRAYSCALE)

    if img1 is None:
        raise FileNotFoundError(f"Could not read image: {img1_path}")
    if img2 is None:
        raise FileNotFoundError(f"Could not read image: {img2_path}")

    K1 = build_camera_intrinsics(img1.shape, fov_deg)
    K2 = build_camera_intrinsics(img2.shape, fov_deg)

    sift = cv2.SIFT_create(
        nfeatures=15000,
        contrastThreshold=0.01,
        edgeThreshold=5,
    )

    kp1, des1 = sift.detectAndCompute(img1, None)
    kp2, des2 = sift.detectAndCompute(img2, None)

    # FLANN matcher
    index_params = dict(algorithm=1, trees=5)
    search_params = dict(checks=50)
    flann = cv2.FlannBasedMatcher(index_params, search_params)
    raw_matches = flann.knnMatch(des1, des2, k=2)

    good = []
    for m, n in raw_matches:
        if m.distance < match_limit * n.distance:
            good.append(m)

    pts1_px = np.float32([kp1[m.queryIdx].pt for m in good])
    pts2_px = np.float32([kp2[m.trainIdx].pt for m in good])

    # ------------------------------------------------------------------ #
    # Stage 1: in-plane transform (X, Y, yaw) via partial affine
    # ------------------------------------------------------------------ #
    M, inlier_mask = cv2.estimateAffinePartial2D(
        pts1_px, pts2_px,
        method=cv2.RANSAC,
        ransacReprojThreshold=DEFAULT_RANSAC_THRESHOLD,
    )
    if M is None:
        raise RuntimeError("estimateAffinePartial2D failed — not enough matches.")

    inlier_matches = [m for m, keep in zip(good, inlier_mask.flatten()) if keep]
    inlier_matches = sorted(inlier_matches, key=lambda m: m.distance)

    # Decompose the 2×3 affine matrix
    # M = [[s·cosθ, -s·sinθ, tx],
    #      [s·sinθ,  s·cosθ, ty]]
    tx_px = M[0, 2]
    ty_px = M[1, 2]
    scale = np.sqrt(M[0, 0]**2 + M[1, 0]**2)   # uniform scale (should be ~1 for same-height)
    theta_rad = np.arctan2(M[1, 0], M[0, 0])

    # ------------------------------------------------------------------ #
    # Stage 2: Z from scale change
    #   scale > 1  → features appear larger in img2 → camera moved closer → ΔZ < 0
    #   scale < 1  → features appear smaller        → camera moved farther → ΔZ > 0
    #   ΔZ (in pixels) = camera_height_px * (1/scale − 1)
    # ------------------------------------------------------------------ #
    camera_height_px = camera_height / pixel_to_um
    dz_px = camera_height_px * (1.0 / scale - 1.0)

    # ------------------------------------------------------------------ #
    # Pack into a 4×4 homogeneous transform for downstream use
    # ------------------------------------------------------------------ #
    R_yaw = np.array([
        [ np.cos(theta_rad), -np.sin(theta_rad), 0],
        [ np.sin(theta_rad),  np.cos(theta_rad), 0],
        [0,                   0,                  1],
    ])
    T = np.eye(4)
    T[:3, :3] = R_yaw
    T[0,  3]  = tx_px
    T[1,  3]  = ty_px
    T[2,  3]  = dz_px

    return img1, img2, kp1, kp2, inlier_matches, T, K1, K2


def decompose_transform(T: np.ndarray, pixel_to_um: float):
    """
    Extract translation (X, Y, Z in µm) and in-plane rotation (yaw, degrees)
    from the 4x4 homogeneous transform produced by compare_images.

    T encodes:
      - T[0,3], T[1,3] : lateral translation in pixels  -> converted to um
      - T[2,3]         : Z shift in pixels               -> converted to um
                         (derived from affine scale: dZ = H*(1/s - 1))
      - T[:3,:3]       : pure yaw rotation matrix
    """
    tx_um = T[0, 3] * pixel_to_um
    ty_um = T[1, 3] * pixel_to_um
    tz_um = T[2, 3] * pixel_to_um

    # Yaw from the rotation matrix (rotation around Z axis only)
    yaw_deg = np.degrees(np.arctan2(T[1, 0], T[0, 0]))

    return (tx_um, ty_um, tz_um), yaw_deg


# ------------------------------------------------------------------ #
# Visualisation  (unchanged logic, kept for convenience)
# ------------------------------------------------------------------ #

def visualize_matches(img1, img2, kp1, kp2, matches):
    img1_color = cv2.cvtColor(img1, cv2.COLOR_GRAY2BGR)
    img2_color = cv2.cvtColor(img2, cv2.COLOR_GRAY2BGR)

    h1, w1 = img1_color.shape[:2]
    h2, w2 = img2_color.shape[:2]

    max_h = max(h1, h2)
    max_w = max(w1, w2)

    if h1 < max_h:
        img1_color = cv2.copyMakeBorder(img1_color, 0, max_h - h1, 0, 0, cv2.BORDER_CONSTANT)
    if w1 < max_w:
        img1_color = cv2.copyMakeBorder(img1_color, 0, 0, max_w - w1, 0, cv2.BORDER_CONSTANT)
    if h2 < max_h:
        img2_color = cv2.copyMakeBorder(img2_color, 0, max_h - h2, 0, 0, cv2.BORDER_CONSTANT)
    if w2 < max_w:
        img2_color = cv2.copyMakeBorder(img2_color, 0, 0, max_w - w2, 0, cv2.BORDER_CONSTANT)

    combined = np.hstack((img1_color, img2_color))
    h, w = combined.shape[:2]
    scale = min(MAX_WIDTH / w, MAX_HEIGHT / h, 1.0)
    if scale < 1.0:
        combined = cv2.resize(combined, (int(w * scale), int(h * scale)), interpolation=cv2.INTER_AREA)

    base = combined.copy()
    show_matches = True
    updated_matches = False
    match_to_color = {}

    window_name = "Feature Matches — 3D (m=toggle, q=quit)"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    while True:
        if cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) < 1:
            break

        _, _, win_w, win_h = cv2.getWindowImageRect(window_name)
        bh, bw = base.shape[:2]
        display = cv2.resize(base, (int(bw * win_w / bw), int(bh * win_h / bh)), interpolation=cv2.INTER_AREA)

        if show_matches and not updated_matches:
            left_img_width = win_w / 2
            for m in matches[:100]:
                x1, y1 = kp1[m.queryIdx].pt
                x2, y2 = kp2[m.trainIdx].pt

                dh, dw = display.shape[:2]
                y1s, x1s = dh / h1, (dw / 2) / w1
                y2s, x2s = dh / h2, (dw / 2) / w2

                pt1 = (int(x1 * x1s), int(y1 * y1s))
                pt2_shifted = (int(x2 * x2s + left_img_width), int(y2 * y2s))

                if m.queryIdx not in match_to_color:
                    match_to_color[m.queryIdx] = tuple(np.random.randint(0, 255, 3).tolist())
                color = match_to_color[m.queryIdx]

                cv2.circle(display, pt1, 4, color, -1)
                cv2.circle(display, pt2_shifted, 4, color, -1)
                cv2.line(display, pt1, pt2_shifted, color, 1)

            updated_matches = True

        cv2.imshow(window_name, display)
        key = cv2.waitKey(-1) & 0xFF

        if key == ord('m'):
            show_matches = not show_matches
            updated_matches = False
        if key == ord('q') or key == 27:
            break

    cv2.destroyAllWindows()


# ------------------------------------------------------------------ #
# CLI entry point
# ------------------------------------------------------------------ #

@APP.command()
def compare_images_script(
    img1_path: Path,
    img2_path: Path,
    visualize: bool = True,
    match_limit: float = DEFAULT_MATCH_LIMIT,
    pixel_to_um: float = DEFAULT_PIXEL_UM,
    camera_height_um: float = DEFAULT_CAMERA_HEIGHT_UM,
    fov_deg: float = DEFAULT_FOV_DEG,
):
    img1, img2, kp1, kp2, matches, T, K1, K2 = compare_images(
        img1_path, img2_path, match_limit, camera_height_um, fov_deg, pixel_to_um
    )

    (tx, ty, tz), yaw_deg = decompose_transform(T, pixel_to_um)

    print(f"Matched features (inliers): {len(matches)}")
    print(f"Translation : X = {tx:+.2f} um,  Y = {ty:+.2f} um,  Z = {tz:+.2f} um")
    print(f"Rotation    : yaw = {yaw_deg:+.2f} deg")

    if visualize:
        visualize_matches(img1, img2, kp1, kp2, matches)


if __name__ == "__main__":
    APP()
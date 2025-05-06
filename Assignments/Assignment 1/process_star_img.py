import cv2
import numpy as np
from matplotlib import pyplot as plt


def detect_stars(image_path,radius_threshold=3):
    # Load image in grayscale
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise FileNotFoundError("Image not found or invalid format")

    # Normalize image to range [0, 1]
    norm_img = img.astype(np.float32) / 255.0

    # Apply a threshold to isolate bright spots (tune value if needed)
    _, thresh = cv2.threshold(norm_img, 0.5, 1.0, cv2.THRESH_BINARY)

    # Convert to uint8 for contour detection
    binary = (thresh * 255).astype(np.uint8)

    # Find contours of bright regions (stars)
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    star_data = []

    for cnt in contours:
        # Compute the center and radius of the minimum enclosing circle
        (x, y), radius = cv2.minEnclosingCircle(cnt)

        # Ensure minimum size to remove noise
        if radius < radius_threshold:
            continue

        x, y, r = float(x), float(y), float(radius)

        # Create a mask for current contour to measure brightness
        mask = np.zeros_like(norm_img)
        cv2.drawContours(mask, [cnt], -1, 1, -1)  # Fill the contour

        # Mean brightness inside the contour
        brightness = float(np.mean(norm_img[mask == 1]))

        if brightness < 0.5:
            continue

        star_data.append((x, y, r, brightness))

    return star_data

# main
if __name__ == "__main__":
    stars = detect_stars("stars pics/fr1.jpg")
    x_coords = [s[0] for s in stars]
    y_coords = [s[1] for s in stars]

    print("num of stars:", len(stars) )
    for s in stars:
        print(f"x={s[0]:.1f}, y={s[1]:.1f}, r={s[2]:.1f}, b={s[3]:.2f}")

    # Plot stars as scatter points
    plt.figure(figsize=(8, 8))
    plt.scatter(x_coords, y_coords, s=10, c='white')
    plt.gca().invert_yaxis()  # Invert Y to match image coordinate system
    plt.gca().set_facecolor('black')
    plt.title("Star Positions (x, y)")
    plt.xlabel("x")
    plt.ylabel("y")
    plt.axis("equal")
    plt.grid(False)
    plt.show()
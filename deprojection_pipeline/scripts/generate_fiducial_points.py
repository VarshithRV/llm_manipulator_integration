# generate 5 circles on a white background, of color RED, Blue, Green, Yellow, and Black
from PIL import Image, ImageDraw

# Define the size of the image to fill a Word document page
width, height = 1654, 2339  # Size for A4 page at 150 DPI
num_circles = 5  # Number of circles
circle_radius = 200  # Radius of each circle
circle_colors = ["red", "blue", "green", "yellow", "black"]  # Colors of the circles
circle_centers = [[500,500], [1200,500], [500,1300], [1200,1300], [1654/2,1900]]

# Create a new image with white background
image_large = Image.new("RGB", (width, height), "white")
draw_large = ImageDraw.Draw(image_large)

# Draw the circles
for i in range(num_circles):
    circle_center = circle_centers[i]
    circle_color = circle_colors[i]
    draw_large.ellipse([circle_center[0] - circle_radius, circle_center[1] - circle_radius,
                        circle_center[0] + circle_radius, circle_center[1] + circle_radius],
                       fill=circle_color)

# Draw the center of size 10 pixels for each circle in white color, for the yellow circle, draw a black center
for i in range(num_circles):
    circle_center = circle_centers[i]
    if i == 3:
        draw_large.ellipse([circle_center[0] - 5, circle_center[1] - 5,
                            circle_center[0] + 5, circle_center[1] + 5],
                           fill="black")
    else:
        draw_large.ellipse([circle_center[0] - 5, circle_center[1] - 5,
                            circle_center[0] + 5, circle_center[1] + 5],
                           fill="white")

# Save the image
image_path_large = "circles.png"
image_large.save(image_path_large)


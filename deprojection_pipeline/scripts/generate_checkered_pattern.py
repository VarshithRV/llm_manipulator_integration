from PIL import Image, ImageDraw

# Draw a checkered pattern of 8x8 squares in an image of A4 size

# Define the size of the image to fill a Word document page
width, height = 1654, 2339  # Size for A4 page at 150 DPI
num_squares = 8  # Number of squares
square_size = width // num_squares  # Size of each square

# Create a new image with white background
image_large = Image.new("RGB", (width, height), "white")
draw_large = ImageDraw.Draw(image_large)

# fill the whole image with the checkered pattern
for i in range(num_squares):
    for j in range(num_squares +4):
        if (i + j) % 2 == 0:
            draw_large.rectangle([i * square_size, j * square_size,
                                  (i + 1) * square_size, (j + 1) * square_size],
                                 fill="black")
            
# Save the image
image_path_large = "checkered_pattern.png"
image_large.save(image_path_large)

from PIL import Image
import csv

# Load the image
image_path = "example.png"
image = Image.open(image_path)

# Get a list of unique colors and their counts
color_counts = image.getcolors(image.size[0] * image.size[1])

# Create a CSV file to store color information
csv_filename = "color_counts.csv"
with open(csv_filename, mode='w', newline='') as csv_file:
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow(["Red", "Green", "Blue", "Count"])  # Write header

    # Write color data to the CSV file
    for count, color in color_counts:
        red, green, blue = color
        csv_writer.writerow([red, green, blue, count])

print("Color information has been written to", csv_filename)

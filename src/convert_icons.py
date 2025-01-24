from PIL import Image

# Open the source image in JPEG format
image = Image.open("tinySA.ico")

# Convert and save the image in PNG format
image.save("tinySA.icns")
print("Image saved successfully in icns format...")

"""
convert_to_gif.py
Converts all frame_*.png images in the output directory into an animated GIF.
Usage: python convert_to_gif.py [--duration 500] [--output animation.gif]
"""

import glob
import os
import argparse
from PIL import Image

def create_gif(input_dir="output", output_file="animation.gif", duration=500, loop=0):
    """
    Create an animated GIF from frame images.
    
    Args:
        input_dir: Directory containing frame_*.png files
        output_file: Output GIF filename
        duration: Time per frame in milliseconds (default 500)
        loop: Number of loops (0 = infinite)
    """
    
    # Find all frame images
    pattern = os.path.join(input_dir, "frame_*.png")
    files = sorted(glob.glob(pattern))
    
    if not files:
        print(f"No frame images found in {input_dir}/")
        print("Expected files like: frame_000.png, frame_001.png, ...")
        return
    
    print(f"Found {len(files)} frames")
    
    # Load images
    images = []
    for filename in files:
        try:
            img = Image.open(filename)
            images.append(img)
            print(f"  Loaded: {os.path.basename(filename)} ({img.size})")
        except Exception as e:
            print(f"  Error loading {filename}: {e}")
    
    if not images:
        print("No valid images loaded.")
        return
    
    # Resize if images are too large (optional, for smaller file size)
    max_dim = 800
    if images[0].width > max_dim or images[0].height > max_dim:
        ratio = max_dim / max(images[0].width, images[0].height)
        new_size = (int(images[0].width * ratio), int(images[0].height * ratio))
        images = [img.resize(new_size, Image.Resampling.LANCZOS) for img in images]
        print(f"  Resized to {new_size}")
    
    # Save as animated GIF
    output_path = os.path.join(input_dir, output_file)
    images[0].save(
        output_path,
        save_all=True,
        append_images=images[1:],
        duration=duration,      # Time per frame in ms
        loop=loop,              # 0 = infinite loop
        optimize=True           # Reduce file size
    )
    
    file_size = os.path.getsize(output_path) / (1024 * 1024)  # MB
    print(f"\nGIF saved: {output_path}")
    print(f"File size: {file_size:.2f} MB")
    print(f"Frames: {len(images)} | Duration: {duration}ms per frame")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert frame images to animated GIF")
    parser.add_argument("--input", default="output", help="Directory containing frame images")
    parser.add_argument("--output", default="animation.gif", help="Output GIF filename")
    parser.add_argument("--duration", type=int, default=500, help="Time per frame in ms")
    parser.add_argument("--loop", type=int, default=0, help="Number of loops (0=infinite)")
    
    args = parser.parse_args()
    create_gif(args.input, args.output, args.duration, args.loop)
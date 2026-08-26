#!/usr/bin/env python3
"""Generate deterministic traffic-sign textures for the Gazebo world."""

from math import cos, pi, sin
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


SIZE = 1024
OUTPUT_DIR = Path(__file__).resolve().parents[1] / "materials" / "textures"
WHITE = (255, 255, 255, 255)
BLACK = (20, 20, 20, 255)
RED = (205, 25, 32, 255)
BLUE = (0, 85, 180, 255)


def canvas():
    return Image.new("RGBA", (SIZE, SIZE), (0, 0, 0, 0))


def regular_polygon(cx, cy, radius, sides, rotation):
    return [
        (
            cx + radius * cos(rotation + 2.0 * pi * index / sides),
            cy + radius * sin(rotation + 2.0 * pi * index / sides),
        )
        for index in range(sides)
    ]


def centered_text(draw, xy, text, font, fill):
    box = draw.textbbox((0, 0), text, font=font, stroke_width=0)
    width = box[2] - box[0]
    height = box[3] - box[1]
    draw.text(
        (xy[0] - width / 2, xy[1] - height / 2 - box[1]),
        text,
        font=font,
        fill=fill,
    )


def turn_left():
    image = canvas()
    draw = ImageDraw.Draw(image)
    draw.ellipse((52, 52, 972, 972), fill=BLUE, outline=WHITE, width=26)

    # Vietnamese mandatory-left sign: a bold white left arrow.
    arrow = [
        (160, 512),
        (430, 280),
        (430, 420),
        (805, 420),
        (805, 604),
        (430, 604),
        (430, 744),
    ]
    draw.polygon(arrow, fill=WHITE)
    return image


def speed_limit_20():
    image = canvas()
    draw = ImageDraw.Draw(image)
    draw.ellipse((52, 52, 972, 972), fill=WHITE, outline=RED, width=112)
    font = ImageFont.truetype(
        "/usr/share/fonts/truetype/dejavu/DejaVuSansCondensed-Bold.ttf", 410
    )
    centered_text(draw, (512, 520), "20", font, BLACK)
    return image


def stop():
    image = canvas()
    draw = ImageDraw.Draw(image)
    outer = regular_polygon(512, 512, 475, 8, pi / 8)
    inner = regular_polygon(512, 512, 425, 8, pi / 8)
    draw.polygon(outer, fill=WHITE)
    draw.polygon(inner, fill=RED)
    font = ImageFont.truetype(
        "/usr/share/fonts/truetype/dejavu/DejaVuSansCondensed-Bold.ttf", 275
    )
    centered_text(draw, (512, 515), "STOP", font, WHITE)
    return image


def main():
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    textures = {
        "turn_left.png": turn_left(),
        "speed_limit_20.png": speed_limit_20(),
        "stop.png": stop(),
    }
    for filename, image in textures.items():
        image.save(OUTPUT_DIR / filename, optimize=True)


if __name__ == "__main__":
    main()

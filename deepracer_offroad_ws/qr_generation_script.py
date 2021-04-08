#  pip3 install qrcode[pil]

import qrcode

# QR positioned on left of the DeepRacer
qr = qrcode.QRCode(
    version=1,
    error_correction=qrcode.constants.ERROR_CORRECT_H,
    box_size=200,
    border=1,
)
qr.add_data('DR: {"wp": 1, "p": "l"}')
img = qr.make_image(fill_color="black", back_color="white")
img.save("deepracer_offroad_qr_images/left_wp1.png")
print(f"Printed QR Code")

# QR positioned on right of the DeepRacer
qr = qrcode.QRCode(
    version=1,
    error_correction=qrcode.constants.ERROR_CORRECT_H,
    box_size=200,
    border=1,
)
qr.add_data('DR: {"wp": 1, "p": "r"}')
img = qr.make_image(fill_color="black", back_color="white")
img.save("deepracer_offroad_qr_images/right_wp1.png")
print(f"Printed QR Code")

# Hairpin Right
qr = qrcode.QRCode(
    version=1,
    error_correction=qrcode.constants.ERROR_CORRECT_H,
    box_size=200,
    border=1,
)
qr.add_data('DR: {"wp": 6, "p": "l", "type": "action_start", "delta": [1.0, -0.2], "limit": 25}')
img = qr.make_image(fill_color="black", back_color="white")
img.save("deepracer_offroad_qr_images/hairpin_right_wp6.png")
print(f"Printed QR Code")


# Hairpin Left
qr = qrcode.QRCode(
    version=1,
    error_correction=qrcode.constants.ERROR_CORRECT_H,
    box_size=200,
    border=1,
)
qr.add_data('DR: {"wp": 6, "p": "r", "type": "action_start", "delta": [-1.0, -0.2], "limit": 25}')
img = qr.make_image(fill_color="black", back_color="white")
img.save("deepracer_offroad_qr_images/hairpin_left_wp6.png")
print(f"Printed QR Code")

# Straight on Left side
qr = qrcode.QRCode(
    version=1,
    error_correction=qrcode.constants.ERROR_CORRECT_H,
    box_size=200,
    border=1,
)
qr.add_data('DR: {"wp": 6, "p": "l", "type": "action_start", "delta": [0.0, -0.2], "limit": 35}')
img = qr.make_image(fill_color="black", back_color="white")
img.save("deepracer_offroad_qr_images/straight_left_wp6.png")
print(f"Printed QR Code")

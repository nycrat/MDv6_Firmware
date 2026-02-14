poly = 0x2F

table = []

for i in range(256):
  crc = i
  # calculate all crc from 1-256 for all bytes
  for _ in range (8):
    # if crc & TOPBIT 0x80=0b10000000
    # we & 0xFF to ensure its just bytes
    if crc & 0x80:
      crc = ((crc << 1) ^ poly) & 0xFF
    else:
      crc = (crc << 1) & 0xFF
  table.append(crc)


print(f"const uint8_t CRC_x2F_TABLE[256] = {{")
for i in range(0, 256, 16):
  line = ", ".join(f"0x{table[j]:02X}" for j in range(i, i + 16))
  print(f"    {line},")
print("};")

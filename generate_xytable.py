kMatrixWidth = 28
kMatrixHeight = 43
XYTable = []

for y in range(kMatrixHeight):
    if y % 2 == 0:  # Even rows
        XYTable.extend(range(y * kMatrixWidth, (y + 1) * kMatrixWidth))
    else:  # Odd rows
        XYTable.extend(range((y + 1) * kMatrixWidth - 1, y * kMatrixWidth - 1, -1))

print(XYTable)
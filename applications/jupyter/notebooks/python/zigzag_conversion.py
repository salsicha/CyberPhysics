
# The string "PAYPALISHIRING" is written in a zigzag pattern on a given number of rows like this: 
# (you may want to display this pattern in a fixed font for better legibility)

# P   A   H   N
# A P L S I I G
# Y   I   R
# And then read line by line: "PAHNAPLSIIGYIR"

# Write the code that will take a string and make this conversion given a number of rows:

# string convert(string s, int numRows);



# Input: s = "PAYPALISHIRING", numRows = 3
# Output: "PAHNAPLSIIGYIR"

# Input: s = "PAYPALISHIRING", numRows = 4
# Output: "PINALSIGYAHRPI"
# Explanation:
# P     I    N
# A   L S  I G
# Y A   H R
# P     I



s = "PAYPALISHIRING"
numRows = 3
steps = numRows - 1

output = []

for i in range(numRows):
    output.append([])

count = 0
i = 0
j = 0
down = -1

for char in s:

    while len(output[j]) < i + 1:
        output[j].append("")

    output[j][i] = char

    if count % steps == 0:
        down *= -1

    if down > 0:
        j += 1
    elif down < 0:
        i += 1
        j -= 1

    count += 1

out_out = []
for str_list in output:
    out_out.append("".join(str_list))

final = "".join(out_out)

print(final)
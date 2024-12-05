
# Roman numerals are represented by seven different symbols: I, V, X, L, C, D and M.

# Symbol       Value
# I             1
# V             5
# X             10
# L             50
# C             100
# D             500
# M             1000
# For example, 2 is written as II in Roman numeral, just two one's added together. 
# 12 is written as XII, which is simply X + II. 
# The number 27 is written as XXVII, which is XX + V + II.

# Roman numerals are usually written largest to smallest from left to right. 
# However, the numeral for four is not IIII. 
# Instead, the number four is written as IV. 
# Because the one is before the five we subtract it making four. 
# The same principle applies to the number nine, which is written as IX. 
# There are six instances where subtraction is used:

# I can be placed before V (5) and X (10) to make 4 and 9. 
# X can be placed before L (50) and C (100) to make 40 and 90. 
# C can be placed before D (500) and M (1000) to make 400 and 900.
# Given an integer, convert it to a roman numeral.

# Input: num = 3
# Output: "III"
# Example 2:

# Input: num = 4
# Output: "IV"
# Example 3:

# Input: num = 9
# Output: "IX"


codes = {
    "I": 1,
    "V": 5,
    "X": 10,
    "L": 50,
    "C": 100,
    "D": 500,
    "M": 1000
}
code_list = ["I", "V" ,"X", "L", "C", "D", "M"]
# code_list = code_list[::-1]
code_list.reverse()

num = 3

output = []

def intToRoman(output, num, code, code_num):
    if num == 4:
        output.append("IV")
        num = 0
    elif num == 9:
        output.append("IX")
        num = 0
    elif num == 40:
        output.append("XL")
        num = 0
    elif num == 90:
        output.append("XC")
        num = 0
    elif num == 400:
        output.append("CD")
        num = 0
    elif num == 900:
        output.append("CM")
        num = 0
    elif int(num / code_num) > 0:
        output.append(code * int(num / code_num))
        num = num % code_num
    return num

for code in code_list:
    num = intToRoman(output, num, code, codes[code])

final = "".join(output)
print(final)

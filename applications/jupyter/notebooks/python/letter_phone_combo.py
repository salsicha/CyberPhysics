
# Given a string containing digits from 2-9 inclusive, 
# return all possible letter combinations that the number could represent. 
# Return the answer in any order.

# A mapping of digit to letters (just like on the telephone buttons) is given below. 
# Note that 1 does not map to any letters.

# 2: abc
# 3: def
# 4: ghi
# 5: jkl
# 6: mno
# 7: pqrs
# 8: tuv
# 9: wxyz

# Example 1:
# Input: digits = "23"
# Output: ["ad","ae","af","bd","be","bf","cd","ce","cf"]

# Example 2:
# Input: digits = ""
# Output: []


num_letter_combo = {
    "2": "abc",
    "3": "def",
    "4": "ghi",
    "5": "jkl",
    "6": "mno",
    "7": "pqrs",
    "8": "tuv",
    "9": "wxyz",
}

output = []

digits = "23"

def recurse(digit_index, string, output):
    if digit_index == len(digits):
        output.append(string)
        return

    digit = digits[digit_index]
    letters = num_letter_combo[digit]

    for char in letters:
        recurse(digit_index + 1, string + char, output)

recurse(0, "", output)

print(output)

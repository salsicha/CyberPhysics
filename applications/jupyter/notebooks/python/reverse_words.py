
# Given an input string s, reverse the order of the words.

# A word is defined as a sequence of non-space characters. 
# The words in s will be separated by at least one space.

# Return a string of the words in reverse order concatenated by a single space.

# Note that s may contain leading or trailing spaces or multiple spaces between two words. 
# The returned string should only have a single space separating the words. 
# Do not include any extra spaces.

###

# Input: s = "the sky is blue"
# Output: "blue is sky the"

# Input: s = "  hello world  "
# Output: "world hello"
# Explanation: Your reversed string should not contain leading or trailing spaces.

# Input: s = "a good   example"
# Output: "example good a"
# Explanation: You need to reduce multiple spaces between two words to a single space in the reversed string.

# Input: s = "  Bob    Loves  Alice   "
# Output: "Alice Loves Bob"

# Input: s = "Alice does not even like bob"
# Output: "bob like even not does Alice"


# Reverse every character
# txt = "string"
# output = []
# for char in txt:
#     output.append(char)
# output.reverse()
# output = "".join(output)
# print(output)


# Reverse word order
txt = " the sky is blue "
output = []
output = txt.split()
# output = output[::-1]
output.reverse()
print(output)

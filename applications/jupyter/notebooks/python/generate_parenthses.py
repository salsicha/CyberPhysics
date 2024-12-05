
# Given n pairs of parentheses, write a function to generate all combinations of well-formed parentheses.

# Input: n = 3
# Output: ["((()))","(()())","(())()","()(())","()()()"]

# Input: n = 1
# Output: ["()"]

# Input: n = 2
# Output: ["(())","()()"]

# Constraints:
# 1 <= n <= 8


n = 3

output = []

def recurse(left, right, string):
    if left == 0 and right == 0:
        output.append(string)
        return

    # print(left, right)
    if left > 0:
        recurse(left - 1, right, string + "(")

    if right > left:
        recurse(left, right - 1, string + ")")

recurse(n, n, "")

print(output)

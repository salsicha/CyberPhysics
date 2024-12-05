
# Given a string s, find the length of the longest substring without repeating characters.

# Input: s = "abcabcbb"
# Output: 3
# Explanation: The answer is "abc", with the length of 3.

# Input: s = "bbbbb"
# Output: 1
# Explanation: The answer is "b", with the length of 1.


s = "abcabcbb"

output = []
longest = 0

for char in s:
    if char not in output:
        output.append(char)
    else:
        longest = max(len(output), longest)
        output = []

print(longest)

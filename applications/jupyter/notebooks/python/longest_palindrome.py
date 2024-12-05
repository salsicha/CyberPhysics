
# Given a string s, return the longest palindromic substring in s.

# Input: s = "babad"
# Output: "bab"
# Note: "aba" is also a valid answer.

# Input: s = "cbbd"
# Output: "bb"


s = "babada"

output = []
for char in s:
    output.append(char)

# output.reverse()
s_i = "".join(output[::-1])

max_len = 0

longest_sub = ""

for i in range(len(s)):
    for j in range(len(s)):
        if j > i:
            try:
                out = s_i.index(s[i:j])
                print(s[i:j])
                if len(s[i:j]) > max_len:
                    max_len = len(s[i:j])
                    longest_sub = s[i:j]
            except:
                pass

print("longest_sub: ", longest_sub)
print(max_len)
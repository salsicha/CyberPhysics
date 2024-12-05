
# Given a signed 32-bit integer x, return x with its digits reversed. 
# If reversing x causes the value to go outside the signed 32-bit integer range [-231, 231 - 1], 
# then return 0.

# Assume the environment does not allow you to store 64-bit integers (signed or unsigned).


# Example 1:

# Input: x = 123
# Output: 321
# Example 2:

# Input: x = -123
# Output: -321


x = -123

x_list = [char for char in str(x)]
x_list.reverse()

sign = "+"
if x_list[-1] == "-":
    sign = x_list.pop()

x_list.insert(0, sign)
x_joined = "".join(x_list)

x_int = int(x_joined)

print(x_int)

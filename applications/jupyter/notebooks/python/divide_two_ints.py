
# Given two integers dividend and divisor, 
# divide two integers without using multiplication, division, and mod operator.

# Return the quotient after dividing dividend by divisor.

# The integer division should truncate toward zero, 
# which means losing its fractional part. 
# For example, truncate(8.345) = 8 and truncate(-2.7335) = -2.

# Note: Assume we are dealing with an environment that could only store integers 
# within the 32-bit signed integer range: [−231, 231 − 1]. 
# For this problem, assume that your function returns 231 − 1 when the division result overflows.

# Input: dividend = 10, divisor = 3
# Output: 3
# Explanation: 10/3 = truncate(3.33333..) = 3.

# Input: dividend = 7, divisor = -3
# Output: -2
# Explanation: 7/-3 = truncate(-2.33333..) = -2.

# Input: dividend = 0, divisor = 1
# Output: 0

# Input: dividend = 1, divisor = 1
# Output: 1
 

# Constraints:

# -231 <= dividend, divisor <= 231 - 1
# divisor != 0


dividend = 10
divisor = 3

count = 0
sum = 0

while True:
    if sum + divisor >= dividend:
        print(count)
        break

    count += 1
    sum += divisor



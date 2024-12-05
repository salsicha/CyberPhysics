
# FizzBuzz

# Print integers 1 to N, 
# but print Fizz if an integer is divisible by 3, 
# Buzz if an integer is divisible by 5, 
# and FizzBuzz if an integer is divisible by both 3 and 5.

N = 10

for x in range(N):
    x += 1
    print("x: ", x)
    if x % 3 == 0 and x % 5 == 0:
        print("FizzBuzz")
    elif x % 3 == 0:
        print("Fizz")
    elif x % 5 == 0:
        print("Buzz")

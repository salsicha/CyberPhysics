#!/bin/python3

import math
import os
import random
import re
import sys

#
# Complete the 'CheckCart' function below.
#
# The function is expected to return an INTEGER.
# The function accepts following parameters:
#  1. STRING_ARRAY codeList
#  2. STRING_ARRAY shoppingCart
#

def CheckCart(codeList, shoppingCart):
    # Write your code here

    print("code list: ", codeList)
    print("shopping cart: ", shoppingCart)

    output = []
    
    for code in codeList:
        print("split: ", code.split())
        code_array = code.split()

        response = 0

        for i, fruit in enumerate(shoppingCart):
            code_match = True

            for j, match in enumerate(code_array):
                if i + j >= len(shoppingCart):
                    code_match = False
                elif shoppingCart[i + j] != match:
                    code_match = False

            if code_match:
                response = 1
                break

        output.append(response)

    return output


if __name__ == '__main__':

    codeList = ['orange', 'apple apple', 'banana orange apple', 'banana']
    shoppingCart = ['orange', 'apple', 'apple', 'banana', 'orange', 'apple', 'banana']

    result = CheckCart(codeList, shoppingCart)

    print(str(result) + '\n')



# code list:  ['orange', 'apple apple', 'banana orange apple', 'banana']
# shopping cart:  ['orange', 'apple', 'apple', 'banana', 'orange', 'apple', 'banana']

# return 1 if winner, else 0

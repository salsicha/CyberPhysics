'''
You're building a tool to estimate the cost of various airplane tickets based on the airline, distance and seating class. Your tool must take in this information as a series of inputs (one ticket calculation per line of input) and produce a list of output costs.

Each airline contains its own cost requirements. Ultimately, the airline is only interested in two major components: the space (seating class) you take on the plane, and the distance you fly. You must generate ticket costs using this gathered data:

Airlines: United, Delta, Southwest, LuigiAir

Operating Costs:

 - Economy:  No charge
 - Premium:  $25
 - Business: $50 + $0.25/mile

Per-Airline Prices:

 - Delta charges $0.50/mile
   + OperatingCost
   
 - United charges $0.75/mile
   + OperatingCost
   + $0.10/mile for Premium seats

 - Southwest charges $1.00/mile

 - LuigiAir charges $100 or 2 * OperatingCost, whichever is higher

Keep in mind that, while there are only four airlines listed above, your solution should be able to expand to dozens of individual airlines,  whose ticket cost can be based on arbitrary functions of "Operating Costs", miles, and/or seating class.

You can assume that the input will be provided as a list of strings and that there could be millions of lines of input. Each string will provide the Airline, Distance and Seating Class. Please review the examples below:

Example Input:
-------------------------------------------
United 150.0 Premium
Delta 60.0 Business
Southwest 1000.0 Economy
LuigiAir 50.0 Business
-------------------------------------------

Example Output:
-------------------------------------------
152.50
95.00
1000.00
125.00
-------------------------------------------

Explanation of Output:
-------------------------------------------
152.50      (150.0 * (0.75 + 0.10) + 25)
95.00       (60.0 * (0.50 + 0.25) + 50)
1000.00     (1000.0 * 1.00)
125.00      (100 <= 2 * (50 + 50 * 0.25))
-------------------------------------------
'''

test_input = [
    "United 150.0 Premium",
    "Delta 60.0 Business",
    "Southwest 1000.0 Economy",
    "LuigiAir 50.0 Business"
]


def OperatingCost(seat_class, mileage):
    cost = 0
    if seat_class == "Economy":
        cost = 0.0
    elif seat_class == "Premium":
        cost = 25.0
    elif seat_class == "Business":
        cost = 50.0 + .25 * mileage
    return cost


def DeltaCost(operating_cost, mileage, seat_class):
    total_cost = mileage * 0.5 + operating_cost
    return total_cost


def UnitedCost(operating_cost, mileage, seat_class):
    cost = 0.75 * mileage
    cost += operating_cost
    if seat_class == "Premium":
        cost += 0.1 * mileage
    return cost

def SouthwestCost(operating_cost, mileage, seat_class):
    return float(mileage)

def LuigiAirCost(operating_cost, mileage, seat_class):
    cost = operating_cost * 2
    if cost < 100.0:
        cost = 100.0
    return cost

if __name__ == "__main__":
    
    airline_cost = {"Delta": DeltaCost, "United": UnitedCost, "Southwest": SouthwestCost, "LuigiAir": LuigiAirCost}
        
    for line in test_input:
        ticket_arr = line.split(" ")
        print(ticket_arr)

        airline = ticket_arr[0]
        mileage = float(ticket_arr[1])
        seat_class = ticket_arr[2]
        operating_cost = OperatingCost(seat_class, mileage)

        ticket_cost = airline_cost[airline](operating_cost, mileage, seat_class)

        print(airline, ticket_cost)

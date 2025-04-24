from pi5RC import pi5RC
pwm0 = pi5RC(12)
pwm1 = pi5RC(13)
pwm2 = pi5RC(18)
pwm3 = pi5RC(19)

# set all r/c to the middle
# the set command put timing in microsecond
#
#
pwm0.set(1500)
pwm1.set(1500)
pwm2.set(1500)
pwm3.set(1500)


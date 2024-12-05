

def sin(angle, factors):
	response = angle

	pow_list = angle
	fac_list = 1

	for term in range(factors):
		factor = term * 2 + 3

		pow_val = pow_last * pow(angle, 2)
		fac_val = fac_last * factor * (factor - 1)

		if term % 2 != 0:
			reponse = reponse + pow_val / fac_val
		else:
			reponse = reponse - pow_val / fac_val

		pow_last - pow_val
		fac_last = fac_val


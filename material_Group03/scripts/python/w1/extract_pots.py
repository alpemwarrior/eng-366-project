with open("pots.txt") as p:
	for l in p.readlines():
		if l.strip().startswith("translation"):
			point = l.strip().split(" ")[1:-1]
			point = [float(x) for x in point]
			print(f"{point[0]}, {point[1]}")



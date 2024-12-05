from rtx import EfficientNetFilm

model = EfficientNetFilm("efficientnet-b0", 10)

out = model("img.jpeg")
from vision import visionserver
import yaml

path = 'vision.yml'
with open(path, 'r') as file:
    data = yaml.safe_load(file)
print(data)
vs = visionserver.ThreadedVisionServer('', 5802, path, data)
vs.listen()

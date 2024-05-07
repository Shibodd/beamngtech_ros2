def ndict_get(data, keys):
  for i, key in enumerate(keys[:-1]):
    if key.endswith("[]"):
      return [
        ndict_get(x, keys[i+1:])
        for x in data[key.removesuffix("[]")]
      ]
    data = data[key]

  return data[keys[-1].removesuffix("[]")]

def ndict_set(data, value, keys):
  for i, key in enumerate(keys[:-1]):
    if key.endswith("[]"):
      key = key.removesuffix("[]")
      
      if key not in data or not isinstance(data[key], list):
        data[key] = [{} for _ in value]

      for j, x in enumerate(value):
        if len(data[key]) <= j:
          data[key].append({})
        elif not isinstance(data[key][j], dict):
          data[key][j] = {}

        ndict_set(data[key][j], x, keys[i+1:])
      return
    
    if key not in data:
      data[key] = {}

    data = data[key]

  data[keys[-1].removesuffix("[]")] = value

class Workspace:
  def __init__(self):
    self.data = {}

  def __parse_path(self, path):
    return [x.strip() for x in path.split(".")]

  def __getitem__(self, path):
    return ndict_get(self.data, self.__parse_path(path))

  def __setitem__(self, path, value):
    ndict_set(self.data, value, self.__parse_path(path))
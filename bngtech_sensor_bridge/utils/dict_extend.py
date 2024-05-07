def dict_extend(dict, source_dict):
  for k, values in source_dict.items():
    if k not in dict:
      dict[k] = values
    else:
      dict[k].extend(values)

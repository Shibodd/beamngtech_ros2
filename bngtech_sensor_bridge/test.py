

def register(a):
  return lambda fn: fn

@register(1)
def fn():
  return 0

print(fn())
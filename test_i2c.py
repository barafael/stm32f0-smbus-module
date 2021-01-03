from smbus2 import SMBus

bus = SMBus(1)

ADDRESS = 0x19

bus.write_byte(ADDRESS, 0xaa)
b = bus.read_byte(ADDRESS)
assert b == 0xaa

print("Read byte and write byte OK")

b = bus.read_byte_data(ADDRESS, 1)
assert b == 0xaa

print("Read byte data@1 OK")

bus.write_byte_data(ADDRESS, 4, 0x44)
b = bus.read_byte_data(ADDRESS, 1)
assert b == 0x44
print("Write byte data@4 OK")

bus.write_byte_data(ADDRESS, 5, 0x55)
b = bus.read_byte_data(ADDRESS, 2)
assert b == 0x55
print("Write byte data@5 OK")

bus.write_byte_data(ADDRESS, 6, 0x66)
b = bus.read_byte_data(ADDRESS, 3)
assert b == 0x66
print("Write byte data@6 OK")

bus.write_word_data(ADDRESS, 9, 0xabcd)
w = bus.read_word_data(ADDRESS, 7)
assert w == 0xabcd
print("Write word data@9, read word data@7 OK")

bus.write_word_data(ADDRESS, 10, 0xef12)
w = bus.read_word_data(ADDRESS, 8)
assert w == 0xef12
print("Write word data@10, read word data@8 OK")

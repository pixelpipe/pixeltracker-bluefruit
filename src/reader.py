
class BufferedRead():
    def __init__(self, uart):
        self.counter = 1
        self.buffer = bytearray()
        self.uart = uart

    def read(self):
        data = self.uart.read(1)
        self.buffer.append(data[0])
        self.counter -= 1
        return self.counter == 0

    def arm(self, count):
        self.counter = count
        
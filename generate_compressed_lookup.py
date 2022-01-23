from enum import Enum

# Generic Sony DVD player controls
class PS2_button_0(Enum):
  _1 = 0x00
  _2 = 0x01
  _3 = 0x02
  _4 = 0x03
  _5 = 0x04
  _6 = 0x05
  _7 = 0x06
  _8 = 0x07
  _9 = 0x08
  _0 = 0x09
  OK = 0x0B
  Return = 0x0E
  Clear = 0x0F
  Menu_top = 0x1A
  Menu_pop_up = 0x1B
  Time = 0x28
  Repeat = 0x2C
  Power = 0x2E
  Previous = 0x30
  Next = 0x31
  Play = 0x32
  Reverse = 0x33
  Forward = 0x34
  Shuffle = 0x35
  Stop = 0x38
  Pause = 0x39
  Display = 0x54
  Slow_reverse = 0x60
  Slow_forward = 0x61
  Subtitle = 0x63
  Audio = 0x64
  Angle = 0x65
  Up = 0x79
  Down = 0x7A
  Left = 0x7B
  Right = 0x7C

# PS2-specific controls
class PS2_button_1(Enum):
  Playstation = 0x15
  Eject = 0x16
  Power_on = 0x2E
  Power_off = 0x2F
  Select = 0x50
  L3 = 0x51
  R3 = 0x52
  Start = 0x53
  Up = 0x54
  Right = 0x55
  Down = 0x56
  Left = 0x57
  L2 = 0x58
  R2 = 0x59
  L1 = 0x5A
  R1 = 0x5B
  Triangle = 0x5C
  Cross = 0x5E
  Circle = 0x5D
  Square = 0x5F

# Full list takes up 70 bytes
# Some entries are commented out because the TV reacts to them when in external input mode
test = {
    # Turns off TV 0x0c: 0x0c, # pwr
    0x31: "S Stop", # stop
    0x30: "S Pause", # pause
    # Opens recording menu 0x37: 0x37, # rec
    0x2b: "S Previous", # rew
    0x2c: "S Play", # play
    0x28: "S Next", # ff
    # 0x8f: 0x8f, # ambi
    # 0xd2: 0xd2, # list
    # 0xbf: 0xbf, # sett
    # 0xcc: 0xcc, # guide
    # Opens help menu 0xb4: 0xb4, # search
    # 0x74: 0x74, # picks
    # 0x38: 0x38, # src
    # Opens apps 0x57: 0x57, # apps
    # Opens TV 0x9f: 0x9f, # tv
    0x6d: "P Square", # red
    0x6e: "P Triangle", # green
    0x6f: "P Start", # yellow
    # Brings up help menu 0x70: 0x70, # blue
    # 0x0f: 0x0f, # info
    0x58: "P Up", # up
    # 0x40: 0x40, # plus
    0x5a: "P Left", # left
    0x5c: "P Cross", # ok
    0x5b: "P Right", # right
    0x0a: "P Circle", # back
    0x59: "P Down", # down
    # 0x54: 0x54, # home
    # 0x10: 0x10, # vol+
    # 0x76: 0x76, # netfl
    # 0x20: 0x20, # ch+
    # 0x11: 0x11, # vol-
    # 0x0d: 0x0d, # mute
    # 0x21: 0x21, # ch-
    0x01: "P L1", # 1
    0x02: "S Menu_top", # 2
    0x03: "P R1", # 3
    0x04: "P L2", # 4
    0x05: "S Repeat", # 5
    0x06: "P R2", # 6
    0x07: "P L3", # 7
    0x08: "P Select", # 8
    0x09: "P R3", # 9
    0x4b: "S Audio", # subt
    0x00: "S Subtitle", # 0
    # Decided to use it for "stop listening" 0x3c: 0x3c, # text
    0xAC: "P Power_on", # custom
}

test_int = []
for k, v in test.items():
    if type(v) != int:
        if v.startswith("P "):
            v = PS2_button_1[v.split(' ')[1]].value | 0x80
        elif v.startswith("S "):
            v = PS2_button_0[v.split(' ')[1]].value
        else:
            v = int(v)
        test_int.append((k, v))
    else:
        test_int.append((k, v))

test_int.sort()
assert all([x < 0xE0 for _, x in test_int])

output = [test_int[0][1]]
for i in range(1, len(test_int)):
    diff = test_int[i][0] - test_int[i-1][0]
    if (diff > 1):
        diff -= 1
        while diff > 32:
            output.append(0xFF)
            diff -= 32
        output.append(0xE0 + diff - 1)
    output.append(test_int[i][1])

print("Bytes: ", len(output))
N_IN_ROW = 8
print('\n'.join([''.join(map(lambda y: f'{y}, '.ljust(5), output[x: x + N_IN_ROW])) for x in range(0, len(output), N_IN_ROW)]))

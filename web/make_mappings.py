import csv

bindings = []

for x in range(24*8):
    bindings.append([])
    
    
fixtures = []
for x in range(24):
    fixtures.append([])


with open('/home/bruce/fixtures.csv') as csvfile:
    rows = csv.reader(csvfile)
    for row in rows:
        (location, idstr, num_buttons, button_order) = row
        if location != "Light Fitting" and idstr.isdigit():
            loc = int(idstr)
            num = len(button_order)
            btns = []
            for p in range(num):
                btns.append(int(button_order[p]))
            fixtures[loc] = btns

with open('/home/bruce/switches.csv') as csvfile:
    rows = csv.reader(csvfile)
    for row in rows:
        (name, gang, location, device_str, address_str, button1, button2, button3, note) = row
        if name != "Light" and button1 != "" and device_str != "" and address_str != "":
            device = int(device_str)
            address = int(address_str)
            
            if button1 != "" and button1 != "???":
                sp = button1.split("/")
                fixture = int(sp[0])    
                logical_btn = int(sp[1])
                btnidx = fixtures[fixture][logical_btn]
                btn = fixture*8+btnidx
                bindings[btn].append(device << 8 | address)
            if button2 != "":
                sp = button2.split("/")
                fixture = int(sp[0])
                btnidx = fixtures[fixture][int(sp[1])]
                btn = fixture*8+btnidx
                bindings[btn].append(device << 8 | address)
            if button3 != "":
                sp = button3.split("/")
                fixture = int(sp[0])
                btnidx = fixtures[fixture][int(sp[1])]
                btn = fixture*8+btnidx
                bindings[btn].append(device << 8 | address)
        
        
for idx, binding_list in enumerate(bindings):
    print(",".join([str(x) for x in binding_list]))
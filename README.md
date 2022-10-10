# Dough Proofer

I wanted a super simple[^1] dough proofer utilizing:

- a cooler
- a relay (SSR in my case) to turn on/off heating element(s)
- ESP8266 (Wemos D1 Mini)
- DS18b20 temperature probe (no need for resistor)

This iteration does pull in wifi but just to get NTP. I should clean up some things but I realized I never saved my prior effort with this hardware (a sous vide) and it would be best to commit the working yet not super polished code in order to avoid loosing it.

I'm using this to proof sourdough but it should work for anything. In my case, I use it with a small heating pad and a low wattage incandescent light (warning: fire hazard). But I also plan on using it to make natto (fermented soybeans) and other things.

Initial code based on a DS18b20 example. I didn't take the time to clean it up -- it would be nice if it just used the first probe it found instead of hard coding an address in like I am now but future me will (probably) fix that!

[^1] not really simple but simple for what I already had on hand

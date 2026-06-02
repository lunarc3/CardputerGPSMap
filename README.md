**This is just a toy! Please prioritize professional equipment in emergencies!**

# Author's Ramblings

It started out based on the gpsinfo code, but during actual use I found a few issues with the original and a whole lot of issues of my own: for instance, displaying satellite information over a long period would cause a memory leak. Also, the vector map showing certain bodies of water would turn into a mess of unreadable lines and make the Cardputer extremely laggy (I even spent two days thinking about how to optimize it).

Without PSRAM, programming was a huge struggle. I ran into countless problems.

In the end, I decided to scrap everything and start over, no longer using the GPS info code. Rewrote it from scratch.

Optimized the Cardputer rendering, considered RGB565 but felt the current approach is more storage-friendly, almost rewrote the converter to handle large maps... Some updates are detailed on GitHub, some aren't—I've been sleeping way too late lately.

Because there's no PSRAM, the map display has been pushed to its absolute limit. So if anyone wants modern GPS-style route planning and navigation, I can only hope M5Stack will eventually release a PSRAM upgrade kit for the ADV (if we shout loud enough, maybe it'll happen?).

But I'm also grateful for the lack of PSRAM—it made me notice many things I would never have paid attention to in my usual programming.

Finally, when will Cardputer get a PSRAM upgrade kit? I want PSRAM!
Like, with PSRAM, even a DOS emulator or something would probably be playable.

Thank you for reading all my nonsense :)
There's even more of it in the GitHub commit messages :)

# A Brief Introduction

Doomsday Offline Map! Doesn't that just make it simple and clear what I mean?

PC side: Just download the map file from OpenStreetMap, run it through my magical converter, and throw all the generated files onto an SD card!

Cardputer ADV side: Use whatever flasher you like or M5Launcher, and flash the bin file!

Power it on! (Remember to go outdoors)

The official LoRa hat will give you a much better experience, but you can use it without—it just won't be as nice.

## Cardputer Operation Guide

In map mode,
↑↓←→ to pan around
Esc (` key) to return to your current GPS location
Z to zoom out, X to zoom in

Enter to take a screenshot—if someone wants to post it to Instagram or whatever
Tab to switch screens: GPS info screen and map screen

Intuitive, right?

On startup, it first checks for GPS signal; once satellites are found (the Cardputer's LoRa hat needs about 30 seconds for a cold start), it will automatically switch to map mode.

## Simplest Usage of the Map Converter

PC side:
0. Download a Python interpreter, if you don't already have one.
1. Download the Python-format converter (hosted on GitHub, released separately from the bin file; get the latest one).
2. Go to OpenStreetMap and download the map you want.
3. Run the converter on your PC, and it will do the conversion for you.
   **You need to open the terminal and run `pip install protobuf grpcio-tools numpy Pillow` once first.**
   **For just playing around—even a movie-style doomsday scenario—z12 is basically sufficient. If you want more detail and don't mind larger storage and longer conversion times, feel free to experiment with z13, z14, z15.**
   **Some people might want to use the small map converter (e.g., if your OSM file is only about 100MB); see the notes below. This is optional, the default converter works just as well.**
4. Put the converted files onto an SD card.

## File Storage Notes

Sorry for hogging your SD card space :) If it makes you angry, go ahead and be angry—after all, we don't have PSRAM :'(

- Screenshots are saved by default in the `\gpsmap\screenshot` directory.
- A `\gpsmap\gpsmap.ini` file is automatically created, saving the last GPS information, so that on your next cold boot you'll roughly see the map centered on your last known location.
- Map files are saved by default in the `\gpsmap` folder, for example `\gpsmap\12`.

# If You Want to Know a Bit More

If you have your own ideas about the maps, here's an explanation of the format converter.

First, you need to understand zoom, which I call z (short for zoom).

Here's an example to help you understand the z numbers:

| Zoom Level | Per Tile | Stockholm                                                    | 上海                                                              | New York                                                              |
|------------|----------|--------------------------------------------------------------|-------------------------------------------------------------------|-----------------------------------------------------------------------|
| z6         | ~625km   | Hela Sverige + Östersjön                                     | 中国东部海岸线可见                                                | U.S. East Coast, Maine to Florida                                     |
| z7         | ~312km   | Södra Sverige, de tre stora städerna Stockholm-Göteborg-Malmö | 长三角城市群轮廓，长江入海口                                      | New York–Boston–Washington corridor                                   |
| z8         | ~156km   | Mälaren, Östersjöns öar, E4:an mot Uppsala                   | 上海全市+苏州+杭州湾                                             | New York City's five boroughs + New Jersey + Long Island              |
| z9         | ~78km    | Stockholms innerstad + förorter, skärgården                   | 外环高速、黄浦江蜿蜒、浦东浦西                                    | Hudson River, Manhattan's elongated shape, Central Park               |
| z10        | ~39km    | Kungsholmen, Gamla Stan, Djurgården, E20                     | 内环/南北高架、陆家嘴、世博园                                     | Central Park, Lincoln Tunnel, Brooklyn Bridge                         |
| z11        | ~19km    | Kungsträdgården, Stadshuset, Vasamuseet, Röda linjen         | 人民广场、外滩、东方明珠、世纪公园                                | Times Square, Central Park, Empire State Building, East River         |
| z12        | ~9.7km   | Gamla Stans gränder, Nobelmuseet, Slottet, tunnelbanestation  | 南京路、豫园、陆家嘴三件套                                        | Statue of Liberty, Wall Street, Fifth Avenue, United Nations          |
| z13        | ~4.8km   | Enskilda gator synliga, Stortorget i Gamla Stan               | 外滩每栋建筑轮廓、陆家嘴绿地                                      | Manhattan grid streets, each avenue clearly visible                   |
| z14        | ~2.4km   | Tunnelbanenedgångar, enstaka byggnadsgrupper                  | 豫园九曲桥、外白渡桥                                              | Statue of Liberty base, Empire State Building shadows                 |

Everyday scenes roughly correspond to:

| Zoom Level | Stockholm                                           | 上海                             | New York                                               |
|------------|-----------------------------------------------------|----------------------------------|--------------------------------------------------------|
| z6         | "Jag är i Sverige"                                  | "我在华东"                       | "I'm in the Northeastern U.S."                         |
| z8         | "Åk in till stan via E4"                            | "走沪杭高速进城"                 | "Cross the bridge from New Jersey into Manhattan"      |
| z10        | "I närheten av Kungsholmen"                         | "快到内环了"                     | "Driving from Brooklyn toward Manhattan"               |
| z12        | "Framme i Gamla Stan, letar parkering"              | "南京路附近到了"                 | "Near Wall Street, we've arrived"                      |

Now that you have a basic understanding of what I define as z, let's move on to the next step: Non-interactive Mode.

## Non-interactive Mode

I think a couple of examples will make it clear (for non-interactive mode users):

```bash
# Specify zoom via command line (skips interactive prompt)
python osm2tiles.py input.osm.pbf -z 10-13 -b S,W,N,E

# Don't specify zoom, run interactively (press Enter to use the default 10-12)
python osm2tiles.py input.osm.pbf -b S,W,N,E
```

Reading the map into memory for indexing does consume some memory, but don't worry—I spent a considerable amount of time solving memory usage issues during conversion. However, if your map is ridiculously large (probably not bigger than what I tested with? How many people tested with gigabyte-sized maps?), your computer should have enough RAM.
Reference: a ~50MB map uses at most 2GB of memory during conversion, averaging around 1.7GB.

**If you're using PowerShell, the screen might not update.** Blame Microsoft, not me.
Workaround:
- Either press any key casually and the text will refresh,
- Or run `$PSDefaultParameterValues['Out-Host:Pager'] = ''` first, then `python osm2tiles.py`. Remember this is temporary, so don't close the window after running the first command.

# Small Map Special Edition Converter

- It's extremely lightweight and fast, but the experience for "average users" might not be as friendly—it requires understanding more concepts, and the generated maps may need repeated tweaking to get the desired result, which is why I'm listing it separately.
- Very few performance optimizations are done, which also means it can even run on some older dual-core machines.
- You need to install dependencies first in the terminal: `pip install osmium Pillow`
- It's very simple to use; you can even throw multiple .osm.pbf files at it.
- **If you want to use a different font (ttc or ttf format), just place it in the converter directory and the program will automatically detect it.**
- **If your map file is under 200MB and you have around 16GB of RAM, you don't need to worry too much about memory usage.**
- The maps are slightly blurrier, saving storage to the extreme, but you can adjust the q parameter; for example, the default compression is 75, set it to 100 if you want more clarity.

Non-interactive mode examples:
```bash
python osm2tiles.py input.osm.pbf -z 6-14 -l en -b S,W,N,E -q 75
python osm2tiles.py -z 6-14
```

# Finally

Spent about a whole week on this, going to bed at 2 AM every day lately. Cardputer is so much fun, really addictive.

My Ko-fi (if you feel like supporting me): https://ko-fi.com/lunarc3

Regarding merging multiple maps (for the small map converter), I haven't thought of a more elegant solution yet: currently, it's the ugly approach of locally merging files first, then generating the map all at once. Each map update requires a full rebuild. Maybe? A more elegant solution will come in a future update.

Possible upcoming fix: I haven't fully tested coastlines; they might display abnormally due to OSM data (especially around z6 level).

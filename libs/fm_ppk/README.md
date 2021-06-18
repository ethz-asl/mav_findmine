# fm_ppk
Scripts to perform a PPK survey using rtklib and swiss topo correction services.

# 1. Convert .sbp data to .obs file
```./sbp2rinex piksi/base_station_receiver_0_2020_15_09_18_42_56.sbp -o piksi/piksi.obs```

# 2. Download matching Swisstopo data
2.1 https://shop.swipos.ch/Login.aspx

2.2 Account details https://wiki.asl.ethz.ch/index.php/IT_Accounts_and_Responsibilities

2.3 RINEX shop

2.4 Neue Bestellung

2.5 Continuously Operating Reference Station (CORS)

2.6 Select station

2.7 Weiter: Zeitwahl (see .obs file for start and end time)

2.8 Download and extract observation interval (see swisstopo folder)

# 3. Run rnx2rtkp (RTKLIB >= rtklib_2.4.3)
```~/RTKLIB/app/rnx2rtkp/gcc/rnx2rtkp piksi/piksi.obs swisstopo/ETH2260L00.20o swisstopo/ETH2260L00.20n swisstopo/ETH2260L00.20g swisstopo/ETH2260L00.20c swisstopo/ETH2260L00.20l -k configuration.conf -o hoengg_base.pos```

# 4. Calculate Q1 (best quality) average
```python plot_pos.py ~/data/2020_06_24_spiez/base_station_ppk/0818/spiez_base.pos 46.690715499900513 7.6472965529539341 675.2428477127105```

# 5. Update base station position in Piksi firmware

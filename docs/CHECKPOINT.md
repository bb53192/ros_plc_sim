# Checkpoint — PLC-vođena pick-and-place ćelija (PoC ostvaren)

> **Cilj projekta:** dokazati da **PLC (OpenPLC Runtime)** može sekvencirati robota + Gazebo
> simulaciju preko standardnih industrijskih protokola, kao **ponovno iskoristiv building block**
> na kojem se gradi bilo koja buduća PLC integracija.

---

## 1. Kratki sažetak (TL;DR)

Izgradili smo potpunu robotsku ćeliju u Gazebo Harmonic (FANUC CRX-10iA + Robotiq 2F-85 gripper +
prava fizička traka s graničnikom i kontaktnim senzorom + kutija na postolju) i spojili je pod
nadzor pravog PLC-a (OpenPLC). PLC pokreće cijeli ciklus preko **dva industrijska protokola
istovremeno** — **OPC UA** za handshake (`belt_run`, `robot_start`, `robot_busy`, `cycle_done`) i
**Modbus** za senzor prisutnosti komada (`part_present`) — a povezni sloj je korisnikov
`plc_bridge` (OPC UA klijent + Modbus server). Kad PLC upali traku, komad doputuje do graničnika,
senzor okine, PLC pokrene robota koji odradi pick-and-place u kutiju, javi da je gotov, novi komad
se spawna i loop se vrti neprekidno — **bez ijedne ručne ROS komande**. Robot koristi MoveIt samo
za inverznu kinematiku (`/compute_ik`), a gibanje izvršava `ros2_control` interpolacijom po
ručno kalibriranim waypointima. Time je PoC zatvoren: **simulacija fizike sjedi pod „mozgom" PLC-a
i sluša ga preko istih protokola kojima bi slušala u pravom pogonu.**

---

## 2. Arhitektura

```
┌─────────────┐   OPC UA (named vars)    ┌──────────────┐   ROS 2 topics    ┌──────────────┐
│  OpenPLC    │◄────────────────────────►│  plc_bridge  │◄─────────────────►│   cell_io    │
│  Runtime    │   belt_run, robot_start  │  (OPC UA     │  /plc/*           │ (orchestr.)  │
│  (ST/LAD)   │   robot_busy, cycle_done │   klijent)   │  /plc/write/*     │              │
│             │                          │              │                   │      │       │
│  "MOZAK"    │   Modbus (discrete in)   │  (Modbus     │  /sensors/*       │      ▼       │
│             │◄─────────────────────────│   server)    │◄──────────────────│  MoveIt IK   │
└─────────────┘   part_present (addr 3)  └──────────────┘                   │ /compute_ik  │
                                                                            │      │       │
                                                                            │      ▼       │
                                                                            │ ros2_control │
                                                                            │      │       │
                                                                            │      ▼       │
                                                                            │   Gazebo     │
                                                                            │  (fizika,    │
                                                                            │   senzori)   │
                                                                            └──────────────┘
```

Sve komponente (OpenPLC container, bridge, ROS/sim) vrte se na **host networku** pa komuniciraju
preko `127.0.0.1`.

---

## 3. Što smo izgradili, popravili i dodali

### 3.1 Fizička ćelija (Gazebo) — `worlds/gripper_cell.sdf`
- **Prava traka umjesto lažne.** Prvo je postojala „varljiva" ploča koja je samo pomicala blok.
  Zamijenili smo je **pravom fizičkom trakom** (`belt_track` + gz `TrackController`) koja
  frikcijom (surface velocity) gura komad. Rolne su poravnate s trakom (`radius = belt_height/2`)
  da se komad ne digne na kraju.
- **Graničnik + kontaktni senzor** (`conveyor_end_stop` + `end_contact`). Komad se nasloni na lip,
  a gz Contact sistem javlja kontakt → to je izvor `part_present`.
- **Kutija na postolju** (`collection_box` na 4-nogom postolju), pomaknuta dalje od robota
  (`y −0.45 → −0.65`) da robot ima prirodan doseg.

### 3.2 Gripper — najveći fix (`urdf/robotiq_2f_85_simple_parallel.urdf.xacro`)
- **Problem:** desni prst je bio `<mimic>` zglob. DART (default physics u gz-sim) **ne podržava
  mimic** → pomicao se samo lijevi prst, ništa se nije moglo uhvatiti.
- **Fix:** oba prsta su sad **prava, nezavisno komandirana prizmatična zgloba** (bez mimica),
  vođena zajedno kroz `joint_trajectory_controller` (`gripper_controller`) na iste pozicije.
- **Drugi DART problem:** donji prst (ispod horizontalnog gripera) se zaledio jer DART zaključa
  velocity-servo zglob koji gravitacija pritisne u limit. **Fix:** margina u limitu
  (`lower="-0.002"`) + `friction="5.0"` u dynamics.

### 3.3 MoveIt integracija — `srdf/`, `config/moveit/`
- Novi self-contained MoveIt config za **arm+gripper**: SRDF grupe (`manipulator` chain
  base_link→flange, `gripper` oba prsta), `kinematics.yaml` (KDL), `joint_limits.yaml`,
  `ompl_planning.yaml`, `moveit_controllers.yaml`.
- Launch `crx_gripper_moveit_stack.launch.py` (move_group + RViz) i `crx_gripper_moveit.launch.py`
  (ćelija + MoveIt).

### 3.4 arm_api2 CRX podrška (`arm_api2/config/crx/`)
- Dodali `crx_sim.yaml` + `crx_kinematics.yaml` (mirror UR konfiguracije) i granu u
  `moveit2_simple_iface.launch.py`. (Na kraju se pick ne oslanja na arm_api2 — v. 3.5.)

### 3.5 Pick-and-place — `scripts/crx_pick_place.py` + `config/crx_pick_place_waypoints.yaml`
- **Odluka:** arm_api2 cartesian/OMPL planiranje je znalo zakazati („LIN failed to find plan") i
  tiho ne spustiti ruku. Prešli smo na **deterministički** pristup: `/compute_ik` → direktno
  `arm_controller/follow_joint_trajectory`.
- Waypointi (pick na kraju trake, place u kutiji) su **ručno kalibrirani** u jednom YAML-u.
  Kalibracije iz debugiranja: orijentacija alata (prsti obuhvate kocku po osi Y, dalje od lipa),
  flip za prirodnu forward-posturu (lakat ne udara u traku), pick z spušten na `0.53` (empirijski
  ~4 cm niže od modela sredine hvatišta).

### 3.6 PLC sučelje — `scripts/cell_io.py`
- Orkestrator koji ćeliju izlaže kroz **plain `std_msgs/Bool`** ugovor (v. §5).
- Edge-triggered ciklus (`robot_start` rising edge → jedan pick-place u worker threadu, I/O ostaje
  živ), respawn novog komada na početku trake (gz `set_pose`).
- **Fix `part_present`:** gz contact senzor objavljuje **samo dok kontakt postoji** — kad robot
  digne komad senzor jednostavno prestane slati poruke. Bez fixa je `part_present` ostajao zaglavljen
  na `True` (traka ne kreće, robot lovi nepostojeći komad). Rješenje: čita se **preko timeouta**
  (`present_timeout=0.5s`) — istina samo dok poruke stižu.

### 3.7 Bridge integracija — `plc_bridge` (korisnikov repo)
- **`bridge_node.py` (OPC UA):**
  - URL/kredencijali **konfigurabilni preko env varijabli** (`OPCUA_URL`, `OPCUA_USER`,
    `OPCUA_PASS`); default sad `127.0.0.1:4840`.
  - Podrška za **anonimni** login (prazan `OPCUA_USER`).
  - **Kontinuirani republish stanja** umjesto samo-na-promjenu. Bez toga se `belt_run=True`
    objavio jednom pri startu i izgubio (DDS discovery kasni), pa traka nije kretala dok se bridge
    ručno ne restarta.
- **`modbus_sensor_bridge.py` (Modbus):** dodan `/sensors/part_present` → **discrete input
  adresa 3** (uz postojeće motion=1, door_closed=2).

### 3.8 PLC program (OpenPLC, ST/LAD)
- Function Block `CellCycle` (state machine) + `main` program koji mapira I/O na located varijable
  i instancira FB. State machine: `RUN_BELT → AT_END → PICKING → RUN_BELT`.

---

## 4. Kako radi — korak po korak

Jedan puni ciklus, kroz sve slojeve:

1. **PLC: `belt_run := TRUE`** (stanje RUN_BELT). Varijabla je OpenPLC izlaz izložen preko OPC UA.
2. **Bridge** je OPC UA klijent — pročita `belt_run` i objavi ROS topic `/plc/belt_run = true`.
3. **cell_io** subscribe-a `/plc/belt_run` → objavi `/conveyor/belt_cmd = -0.15` (Float64),
   bridgean u gz → `TrackController` frikcijom pokrene traku.
4. Komad **fizički putuje** trakom do graničnika i nasloni se. gz Contact senzor počne slati
   poruke na `/conveyor_end_sensor/contacts`.
5. **cell_io** vidi kontakt → `/sensors/part_present = true`.
6. **Modbus bridge** (server na :502) upiše taj Bool u **discrete input adresu 3**. OpenPLC ga kao
   Slave Device master pročita → PLC varijabla `part_present = 1`.
7. **PLC: prijelaz RUN_BELT → AT_END:** `belt_run := FALSE` (traka stane), `robot_start := TRUE`.
8. Bridge objavi `/plc/robot_start = true`. **cell_io** detektira **rising edge** → pokrene jedan
   pick-place ciklus u worker threadu i postavi `robot_busy = true`.
9. Za svaki waypoint (`pick_approach → pick → lift → place_approach → place`):
   - cell_io uzme TCP pozu iz YAML-a → **MoveIt `/compute_ik`** (KDL) vrati kutove zglobova
   - kutovi → **`arm_controller`** (JointTrajectoryController) koji **interpolira** glatku
     trajektoriju → **`gz_ros2_control`** pogoni zglobove u Gazebu
   - gripper preko `gripper_controller` (zatvori na kocki, otvori nad kutijom)
10. **cell_io** javi `robot_busy → false`, `cycle_done → true` (preko `/plc/write/*` → bridge OPC
    UA **upiše** te varijable natrag u PLC).
11. **PLC: AT_END/PICKING → RUN_BELT:** na `cycle_done` resetira i vrati `belt_run := TRUE`.
12. cell_io je u međuvremenu **respawnao** novi komad na početku trake; robot lifta komad →
    `part_present` padne na 0 (timeout) → sve spremno za sljedeći krug.

Sve se vrti neprekidno, PLC je jedini koji odlučuje **kada** što ide.

---

## 5. Signalni ugovor (I/O)

| Signal | Smjer | Transport | ROS topic | PLC var |
|---|---|---|---|---|
| `belt_run` | PLC → ćelija | **OPC UA** (read) | `/plc/belt_run` | izlaz (`%QX`) |
| `robot_start` | PLC → ćelija | **OPC UA** (read) | `/plc/robot_start` | izlaz (`%QX`) |
| `robot_busy` | ćelija → PLC | **OPC UA** (write) | `/plc/write/robot_busy` | ulaz |
| `cycle_done` | ćelija → PLC | **OPC UA** (write) | `/plc/write/cycle_done` | ulaz |
| `part_present` | ćelija → PLC | **Modbus** (discrete in) | `/sensors/part_present` | ulaz `addr 3` |

Bridge sam otkrije OPC UA varijable po **browse name-u** i mapira ih na `/plc/<name>` (read) i
`/plc/write/<name>` (write, ako je varijabla writable). Zato imena varijabli u OpenPLC-u moraju
točno odgovarati (`belt_run`, `robot_start`, `robot_busy`, `cycle_done`), a `robot_busy`/`cycle_done`
moraju biti **Read/Write** za korisnika pod kojim se bridge loginira.

---

## 6. Modbus vs OPC UA — što su i zašto oba

### Što je Modbus
Najstariji i najrašireniji industrijski protokol (1979.). Krajnje **jednostavan**: memorija je niz
numeriranih registara/bitova koje čitaš/pišeš po **adresi** i **funkcijskom kodu** (npr. FC2 =
discrete inputs = 1-bitni ulazi „senzora"). **Nema imena, tipova ni semantike** — samo „bit na
adresi 3". Govori ga **doslovno svaki** PLC/SCADA/senzor. Mana: nema sigurnosti, nema
samoopisivanja, ručno vodiš mapu adresa.

### Što je OPC UA
Moderni (2008.+), servisno-orijentirani protokol. Adresni prostor su **imenovane, tipizirane
varijable** koje klijent može **otkriti (browse/discovery)**, s ugrađenom **autentikacijom i
enkripcijom** i pravima pristupa po ulozi. Samoopisiv je i puno „pametniji". Mana: **teži**,
zahtjevniji za postaviti (server, useri, endpointi), i **nema ga svaki** (pogotovo jeftiniji/stariji
PLC-ovi).

### Zašto smo koristili OBA (a ne sve preko OPC UA)
- **Handshake preko OPC UA:** `belt_run`, `robot_start`, `robot_busy`, `cycle_done` su logička
  stanja s jasnim imenima. OPC UA ih bridge **automatski otkrije po imenu** i mapira simetrično
  (read + write) — nema ručne mape adresa, nema off-by-one. Zato smo **konsolidirali handshake na
  OPC UA** („da nemamo dodatne probleme bespotrebno").
- **`part_present` preko Modbusa:** namjerno ostavljen na Modbusu jer je to **klasičan uzorak
  „ožičeni senzor kao discrete input"**. Time PoC pokazuje da ćelija zna govoriti i
  **najniži-zajednički-nazivnik** fieldbus koji razumije apsolutno svaki PLC — dakle ćelija je
  **fieldbus-agnostična**, nije vezana samo na moderni protokol.
- **Building-block vrijednost:** imati oba u istom PoC-u demonstrira **dva integracijska uzorka
  odjednom** (imenovani handshake vs. sirovi senzor). Sutra spojiš stariji PLC koji ima samo Modbus,
  ili moderni koji sve vozi preko OPC UA — obje putanje su već dokazane.

> Zaključak: OPC UA = „pametni, imenovani razgovor" za logiku; Modbus = „univerzalni sirovi bit" za
> senzore. Realne integracije često miješaju baš tako.

---

## 7. Integracija simulator ↔ PLC: što se dobije i gdje su uska grla

### Što se dobije
- **Validacija upravljačke logike prije hardvera.** Sekvenca, interlockovi, I/O mapiranje i timing
  se testiraju protiv **prave fizike** i **pravog PLC-a**, bez rizika i troška stvarne stanice.
  Simulacija je stand-in za fizičku ćeliju.
- **Isti protokoli kao u pogonu.** PLC ne zna (i ne treba znati) da je s druge strane simulacija —
  govori OPC UA/Modbus kao i sa stvarnim robotom. Znači logika se prenosi 1:1 na hardver.
- **Ponovno iskoristiv kostur.** Bridge + `cell_io` ugovor + PLC state machine su generički; nova
  ćelija = novi waypointi + novi signali, ne novi sustav.

### Uska grla (bottlenecks)
- **Poll latencija bridgea.** OPC UA se poll-a na ~10 Hz, Modbus slično. To je **sasvim dovoljno za
  sekvenciranje**, ali nije za hard-realtime upravljanje gibanjem (to ostaje u ros2_control/robotu).
- **State vs. change semantika.** „Objavi samo na promjenu" gubi poruke ako se subscriber spoji
  kasno (izgubljeni `belt_run=True`). Riješeno kontinuiranim republishom — ali to je klasična zamka
  bridge dizajna.
- **Semantika senzora.** gz contact senzor šalje poruke samo dok traje kontakt → latch na `True`.
  Riješeno timeoutom. Svaki senzor treba pažljivo mapirati level vs. edge vs. „tišina".
- **Nema collision-aware planiranja.** Putanje su fiksni kalibrirani waypointi; ako se geometrija
  ćelije promijeni, treba rekalibrirati (ili uvesti pravi MoveIt OMPL planer).
- **Mrežni/auth sloj.** Host network, portovi, OPC UA useri/prava, jednokratni discovery — realno
  integracijsko trenje (v. §9 lekcije).
- **Kadenca jednog komada.** Nema reda čekanja / više komada odjednom — jedan ciklus u trenutku.

---

## 8. PLC: kako usporava integraciju — i zašto je ipak neophodan

### Kako usporava (cijena)
PLC je **dodatni sloj** koji integraciju čini sporijom i osjetljivijom:
- Moraš **mapirati signale na protokol i adrese** (OPC UA imena, Modbus adrese) i držati **PLC
  program + bridge + ćeliju sinkronizirane** — promjena imena varijable ruši lanac.
- **Mrežna konfiguracija** (host network, portovi 4840/502, OPC UA useri i Read/Write prava,
  jednokratni discovery koji zahtijeva da varijable postoje prije spajanja).
- **Debugiranje se širi na tri domene** — ST/LAD u PLC-u, Python bridge, ROS graf. Bug može biti u
  edge-triggeru, u QoS-u, u semantici senzora ili u PLC state machineu.
- **Timing i handshake moraju biti točni** — rising edge, reset `robot_start`, level vs. pulse. Sve
  smo to morali precizno posložiti da loop ne zapne.

### Zašto je neophodan (mozak)
Unatoč cijeni, PLC je **razlog zašto ovo uopće radimo** — jer u stvarnom pogonu:
- **On je deterministički, uvijek-upaljeni nadzornik** koji sekvencira ćeliju. Robot/sim samo
  **izvršava**; PLC odlučuje **KADA** i **provodi logiku** (upali traku, čekaj senzor, pokreni
  robota, čekaj gotovo, ponovi).
- **On je jedini izvor istine o stanju ćelije** i **preživljava** restarte ROS-a/robota. Ugasiš i
  upališ sim — PLC i dalje zna u kojem je koraku.
- **On provodi interlockove i sigurnost**, koordinira **više strojeva** na liniji i **standardno je
  sučelje** koje očekuju SCADA, operateri i ostatak pogona. Nitko u industriji ne pušta robota da
  sam sebi bude gospodar bez nadređene deterministične logike.
- Zato je cijeli PoC vrijedan: **dokazali smo da simulacija fizike može sjesti pod taj mozak** i
  slušati ga preko istih protokola kao pravi hardver. To je temelj na koji se dalje kači bilo koja
  PLC integracija.

> Ukratko: PLC te **usporava u integraciji** (više slojeva, mapiranja, mreže), ali te
> **ubrzava u pogonu** (deterministična, nadzorna, standardna logika koja upravlja cijelom
> stanicom). Cijena integracije se plaća jednom; nadzor mozga traje cijeli životni vijek stanice.

---

## 9. Ključne lekcije / gotchas (da se ne ponavljaju)

- **OPC UA server u OpenPLC-u se mora ručno upaliti** (nije default); tek tad sluša na 4840.
- **Auth:** bridge se mora ulogirati kao **valjani user** (`user` / `1234`), `admin/1234` je odbijen
  (`BadUserAccessDenied`). Feedback varijable moraju biti **Read/Write** za tog usera.
- **Sve na host networku** → komunikacija preko `127.0.0.1`; različiti containeri bez host networka se
  ne vide preko localhosta.
- **Bridge mora kontinuirano republishati stanje** (ne samo na promjenu) zbog DDS discovery race-a.
- **gz contact senzor = level dok traje kontakt, tišina inače** → čitaj preko timeouta.
- **DART ne podržava `<mimic>`** → oba prsta gripera moraju biti prava zgloba.

---

## 10. Kako pokrenuti

```bash
# T1 — simulacija (Gazebo cell + MoveIt + cell_io)
source /opt/ros/jazzy/setup.bash && source ~/arms_ws/install/setup.bash
ros2 launch ros_plc_sim crx_gripper_plc.launch.py

# T2 — OPC UA bridge (handshake)
OPCUA_USER=user OPCUA_PASS=1234 ros2 run plc_bridge bridge_node

# T3 — Modbus bridge (part_present -> discrete input 3)
ros2 run plc_bridge modbus_sensor_bridge
```
U OpenPLC-u: upali OPC-UA server; dodaj Modbus Slave Device `127.0.0.1:502`, discrete input adr. 3 =
`part_present`; pokreni PLC program. Loop kreće sam.

---

## 11. Sljedeći koraci (opcije)
- **M6 — percepcija:** kamera/RGBD u Gazebu → detekcija poze komada → dinamički pick waypoint
  (umjesto fiksne koordinate). Prvo ground-truth verzija, pa pravi OpenCV/depth detektor.
- **Collision-aware planiranje** (MoveIt OMPL) ako se geometrija ćelije mijenja.
- **Više komada / red čekanja**, brojači, statistika ciklusa u PLC-u.
- **Sigurnosni interlockovi** (e-stop, vrata) kroz PLC.

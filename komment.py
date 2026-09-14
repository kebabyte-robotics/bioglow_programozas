from pybricks.hubs import * #beimportálja az agyat
from pybricks.pupdevices import * #beimportálja a motorokat
from pybricks.parameters import * #beimportálja a paramétereket
from pybricks.tools import * #beimportálja a toolsokat
from umath import * #beimportálja a matekot a bezier görbéhez
from pybricks.robotics import DriveBase #beimportálja a DriveBase-t a bezier görbéhez
 
hub = PrimeHub() #az agyat elnevezi hubnak
bal  = Motor(Port.B) #a motort ami a B portba van elnevezzük balnak és óra járásával megegyező irányba forog
jobb = Motor(Port.F, Direction.COUNTERCLOCKWISE) #a motort ami az F portba van elnevezzük balnak és óra járásával ellentétes irányba forog
feltet_bal = Motor(Port.A) #a feltét motort ami az E portba van elnevezzük feltet_balnak és a óra járásával megegyező irányba forog
feltet_jobb = Motor(Port.E)  #a feltét motort ami az A portba van elnevezzük feltet_jobbnak és a óra járásával megegyező irányba forog
feltet_bal.control.limits(1000, 10000) #beállítjuk a feltet_balnak a limitjeit: speed, acceleration
feltet_jobb.control.limits(1000, 6500) #beállítjuk a feltet_jobbnak a limitjeit: speed, acceleration
bal.control.limits(2000, 5500) #beállítjuk a balnak a limitjeit: speed, acceleration
jobb.control.limits(2000, 5000) #beállítjuk a jobbnak a limitjeit: speed, acceleration
db = DriveBase(bal, jobb, wheel_diameter=56, axle_track=110) #a DriveBaset elnevezzük dbnek és meg adjuk neki a 2 motort, majd a kerék átmérőt és a kerekek közti távolságot
db.use_gyro(True) #bekapcsolja a dbnél a gyroszkóp alapú vezérlést

while not hub.imu.ready(): #ameddig a gyro nincs kalibrálva/kész
    hub.display.char("x") #addig az agy írjon ki egy x-et
 
hub.imu.reset_heading(0) #a gyro értékét 0-ra állítjuk
irany = 0 #létrehozunk egy irany nevű változót aminek 0 értéket adunk és ez később azt jelzi, hogy merre fel kéne néznie a robotnak
elindult_timer = False #létrehozunk egy elindult_timer nevű változót aminek False értéket adunk és ez később azt jelzi, hogy a meccset időzítő timer elindult már
 
async def egyenes(tavolsag, legkisebb_sebesseg=40, gyorsitas=40, korekcio=0.01, legnagyobb_sebesseg = 700, lassitas=80, timeout = None): #létrehozunk egy egyenes nevü függvényt, paramétereket adunk meg amit használni fogunk a függvényben, az tavolsagot, e_lassitast és a e_gyorsitast mm-be adjuk meg, az alap értékek csak átlagban működnek, azért async, hogy közben más mozgás is le tudjon futni
    if timeout != None: #ha a timeoutnak van értéke
        timeout_watch = StopWatch() #akkor létrehozunk egy timeout_watch nevű stoppert és ez később azt jelzi, hogy mennyi ideig van elakva a robot
        timeout_watch.reset() #akkor lenullázza a stoppert
        timeout_watch.resume() #akkor elindítja a stoppert
    global irany #engedélyezzük a függvénynek az irany változó használatát a függvényen belül
    bal.reset_angle(0) #a bal szögét 0-ra állítjuk
    jobb.reset_angle(0) #a jobb szögét 0-ra állítjuk
    tavolsag = tavolsag / 0.489 #a mm-ben megadott tavolsagot átváltjuk motor fokokra
    gyorsitas /= 0.489 #a mm-ben megadott e_gyorsítástt átváltjuk motor fokokra
    lassitas /= 0.489 #a mm-ben megadott e_lassítást átváltjuk motor fokokra
    while True: #elindítunk egy ciklust ami addig fut ameddig le nem állítjuk
        if timeout != None and timeout_watch.time() >= timeout: #ha van a timeoutnak értéke és a stopper átlépte ezt az értéket
            break #akkor lépjen ki a ciklusból           
        megtett_tavolsag = (bal.angle()+jobb.angle()) / 2 #létrehozunk egy megtett_tavolsag nevű változót aminek jobb és bal motor szögének az átlagát adjuk értéknek és ez később azt jelzi, hogy mennyit haladt a robot
        hatralevo_tavolsag = tavolsag - megtett_tavolsag #létrehozunk egy hatralevo_tavolsag nevű változót aminek tavolsag-megtett_tavolsag értéket adunk és ez később azt jelzi, hogy mennyi távolság van hátra
        jelzo = hatralevo_tavolsag/abs(hatralevo_tavolsag) #létrehozunk egy jelzo nevű változót aminek hatralevo_tavolsag és annak az abszolut értékje osztva értéket adunk és ez később azt jelzi, hogy előre vagy hátra kell mennie a robotnak
        megtett_tavolsag = abs(megtett_tavolsag) #az megtett_tavolsag abszolut értéke legyen az megtett_tavolsag, azért kell hogy pozitiv legyen és a jelző már eltárolta, hogy negativ vagy pozitiv és később pozitivan számolunk vele
        hatralevo_tavolsag = abs(hatralevo_tavolsag) #az hatralevo_tavolsag abszolut értéke legyen az hatralevo_tavolsag, azért kell hogy pozitiv legyen és a jelző már eltárolta, hogy negativ vagy pozitiv és később pozitivan számolunk vele
        if hatralevo_tavolsag < 3: #ha hatralevo_tavolsag kevesebb, mint 3 motorfok(ha már elég közel van a célhoz)
            break #akkor lépjen ki a ciklusból
        if hatralevo_tavolsag < lassitas : #ha az hatralevo_tavolsag kevesebb, mint az lassitas(ha már a lassításba van)
            ratio = hatralevo_tavolsag / lassitas #akkor létrehozunk egy ratio nevű változót aminek hatralevo_tavolsag / lassitas értéket adunk és ez később azt jelzi, hogy milyen gyorsan és melyen mértékbe lassítson 
            mostani_sebesseg = max(ratio * legnagyobb_sebesseg, legkisebb_sebesseg) * jelzo #akkor létrehozunk egy mostani_sebesseg nevű változót aminek a nagyobb értéket adunk jelzo * a kettő közül ratio*legnagyobb_sebesseg vagy legkisebb_sebesseg értéket adunk és ez később azt jelzi, hogy mennyire gyorsan menjen előre
        elif megtett_tavolsag < gyorsitas: #ha nem az előző és a megtett_tavolsag kisebb, mint a gyorsitas(a gyorsitas szakaszban van)
            ratio = megtett_tavolsag / gyorsitas #akkor létrehozunk egy ratio nevű változót aminek megtett_tavolsag / gyorsitas értéket adunk és ez később azt jelzi, hogy hogy milyen gyorsan és melyen mértékbe gyorsitson
            mostani_sebesseg = max(ratio * legnagyobb_sebesseg, legkisebb_sebesseg) * jelzo #akkor létrehozunk egy mostani_sebesseg nevű változót aminek a nagyobb értéket adunk jelzo * a kettő közül ratio*legnagyobb_sebesseg vagy legkisebb_sebesseg értéket adunk és ez később azt jelzi, hogy mennyire gyorsan menjen előre
        else: #ha semelyik előző
            mostani_sebesseg = legnagyobb_sebesseg * jelzo #akkor létrehozunk egy mostani_sebesseg nevű változót aminek jelzo * legnagyobb_sebesseg értéket adunk és ez később azt jelzi, hogy mennyire gyorsan menjen előre
        iranyelteres = (irany - hub.imu.heading()) * korekcio #létrehozunk egy iranyelteres nevű változót aminek a (irany - gyro értéke) * korekcio értéket adunk és ez később azt jelzi, hogy mennyit tévedett a robot
        korekciomertek = mostani_sebesseg * iranyelteres * jelzo #létrehozunk egy korekcio_mertek nevű változót aminek a mostani_sebesseg * iranyelteres * jelzo értéket adunk és ez később azt jelzi, hogy milyen gyorsan és milyen kis mértékekben korigáljon
        bal.run (mostani_sebesseg - korekciomertek) #a bal motoron lefutattják a mostani_sebesseg - korkciomertek
        jobb.run(mostani_sebesseg + korekciomertek) #a jobb motoron lefutattják a mostani_sebesseg + korkciomertek
        await wait(10) #várj 10 millimásodpercet
    bal.stop() #álljon le a bal motor
    jobb.stop() #álljon le a jobb motor
    
 
async def kanyarodas(fok, legnagyobb_sebesseg=360, lassitas=80, legkisebb_sebesseg=50, timeout = None): #létrehozunk egy kanyarodas nevü függvényt, paramétereket adunk meg amit használni fogunk a függvényben, a fokot és e_lassitast motorfokokban-be adjuk meg, az alap értékek csak átlagban működnek, azért async, hogy közben más mozgás is le tudjon futni
    if timeout != None: #ha a timeoutnak van értéke
        timeout_watch = StopWatch() #akkor létrehozunk egy timeout_watch nevű stoppert és ez később azt jelzi, hogy mennyi ideig van elakva a robot
        timeout_watch.reset() #akkor lenullázza a stoppert
        timeout_watch.resume() #akkor elindítja a stoppert
    alap_fok = hub.imu.heading() #létrehozunk egy alap_fok nevű változót aminek a gyro értéket adunk és ez később azt jelzi, hogy ebből számoljuk a tévedést
    global irany #engedélyezzük a függvénynek az irany változó használatát a függvényen belül
    cel_fok = irany+fok #létrehozunk egy cel_fok nevű változót aminek az irany + fok értéket adunk és ez később azt jelzi, hogy ebből számoljuk a tévedést, mert ez adja meg az elméleti fokot
    irany = cel_fok #az irány változónak a cel_fok értéket adjuk meg
    cel_fok -= alap_fok #a cel_fok legyen egyenlő a cel_fok - alap_fok, ez így korigálja hibát
    while True: #elindítunk egy ciklust ami addig fut ameddig le nem állítjuk
        if timeout != None and timeout_watch.time() >= timeout: #ha van a timeoutnak értéke és a stopper átlépte ezt az értéket
            break #akkor lépjen ki a ciklusból  
        megtett_fokok = hub.imu.heading() - alap_fok #létrehozunk egy megtett_fokok nevű változót aminek a gyro - alap_fok értéket adunk és ez később azt jelzi, hogy ebből számoljuk a tévedést, mert ez lesz a megtett távolság
        hatralevo_fokok = cel_fok - megtett_fokok #létrehozunk egy hatralevo_fokok nevű változót aminek a cel_fok - megtett_fokok értéket adunk és ez később azt jelzi, hogy ebből számoljuk a tévedést, mert ez lesz a hátralévő távolság
        jelzo = hatralevo_fokok/abs(hatralevo_fokok) #létrehozunk egy jelzo nevű változót aminek hatralevo_fokok és annak az abszolut értékje osztva értéket adunk és ez később azt jelzi, hogy előre vagy hátra kell mennie a robotnak
        hatralevo_fokok = abs(hatralevo_fokok) #az e_hatralevo_fokok abszolut értéke legyen az e_hatralevo_fokok, azért kell hogy pozitiv legyen és a jelző már eltárolta, hogy negativ vagy pozitiv és később pozitivan számolunk vele
        if hatralevo_fokok <= 0.5: #ha e_hatralevo_fokok kevesebb, mint 3 motorfok(ha már elég közel van a célhoz)
            break #akkor lépjen ki a ciklusból
        if hatralevo_fokok < lassitas : #ha az hatralevo_fokok kevesebb, mint az lassitas(ha már a lassításba van)
            ratio = hatralevo_fokok / lassitas #akkor létrehozunk egy ratio nevű változót aminek hatralevp_fokok / lassitas értéket adunk és ez később azt jelzi, hogy hogy milyen gyorsan és melyen mértékbe lassítson
            mostani_sebesseg = max(ratio * legnagyobb_sebesseg, legkisebb_sebesseg) * jelzo #akkor létrehozunk egy mostani_sebesseg nevű változót aminek a nagyobb értéket adunk jelzo * a kettő közül ratio*legnagyobb_sebesseg vagy legkisebb_sebesseg értéket adunk és ez később azt jelzi, hogy mennyire gyorsan kanyarodjon
        else: #ha nem az előző 
            mostani_sebesseg = legnagyobb_sebesseg * jelzo #akkor létrehozunk egy mostani_sebesseg nevű változót aminek jelzo * legnagyobb_sebesseg értéket adunk és ez később azt jelzi, hogy mennyire gyorsan kanyarodjon
        bal.run(-mostani_sebesseg) #a bal motoron lefutattják a -mostani_sebesseg
        jobb.run(mostani_sebesseg) #a bal motoron lefutattják a mostani_sebesseg
        await wait(10) #várj 10 millimásodpercet
    bal.stop() #álljon le a bal motor
    jobb.stop() #álljon le a jobb motor


async def jobb_feltet(angle, speed=400, timeout=None): #létrehozunk egy jobb_feltet nevü függvényt, paramétereket adunk meg amit használni fogunk a függvényben, a fokot motorfokokban-be adjuk meg, az alap értékek csak átlagban működnek, azért async, hogy közben más mozgás is le tudjon futni
    if timeout != None: #ha a timeoutnak van értéke
        timeout_watch = StopWatch() #akkor létrehozunk egy timeout_watch nevű stoppert és ez később azt jelzi, hogy mennyi ideig van elakva a robot
        timeout_watch.reset() #akkor lenullázza a stoppert
        timeout_watch.resume() #akkor elindítja a stoppert
    feltet_jobb.run_angle(speed, angle, wait=False) #a feltet_jobb motoron lefutattják a speedet és anglet
    while not feltet_jobb.done(): #amig a feltet_jobb nincs kész(nem futott le a mozgás)
        if timeout != None and timeout_watch.time() >= timeout: #ha van a timeoutnak értéke és a stopper átlépte ezt az értéket
            break #akkor lépjen ki a ciklusból  
        await wait(10) #várj 10 millimásodpercet
    feltet_jobb.stop() #álljon le a feltét_jobb motor
 
async def bal_feltet(angle, speed=400, timeout=None): #létrehozunk egy bal_feltet nevü függvényt, paramétereket adunk meg amit használni fogunk a függvényben, a fokot motorfokokban-be adjuk meg, az alap értékek csak átlagban működnek, azért async, hogy közben más mozgás is le tudjon futni
    if timeout != None: #ha a timeoutnak van értéke
        timeout_watch = StopWatch() #akkor létrehozunk egy timeout_watch nevű stoppert és ez később azt jelzi, hogy mennyi ideig van elakva a robot
        timeout_watch.reset() #akkor lenullázza a stoppert
        timeout_watch.resume() #akkor elindítja a stoppert
    feltet_bal.run_angle(speed, angle, wait=False) #a feltet_bal motoron lefutattják a speedet és anglet
    while not feltet_bal.done(): #amig a feltet_jobb nincs kész(nem futott le a mozgás)
        if timeout != None and timeout_watch.time() >= timeout: #ha van a timeoutnak értéke és a stopper átlépte ezt az értéket
            break #akkor lépjen ki a ciklusból 
        await wait(10) #várj 10 millimásodpercet
    feltet_bal.stop() #álljon le a feltét_bal motor

async def bezier-gorbe(p0_x = 0, p0_y = 0, p1_x, p1_y, p2_x, p2_y, p3_x, p3_y, sebesseg=200): #létrehozunk egy bezier-gorbe nevü függvényt, paramétereket adunk meg amit használni fogunk a függvényben, a sebességen kívül mindent koordináta rendszerben adjuk meg, az alap értékek csak átlagban működnek, azért async, hogy közben más mozgás is le tudjon futni, a p0_x és p0_y adja mega  kezdőpont koordinátáit, ami 0, a p3_x és p3_y adja meg a végpont koordinátáit, a többi pedig a vonzópontokat
    global irany #engedélyezzük a függvénynek az irany változó használatát a függvényen belül
    bezier_hossz = 0 #létrehozunk egy bezier_hossz nevű változót aminek 0 értéket adunk és ez később azt jelzi, hogy milyen hosszú a bezier-görbe
    utolso_x = p0_x #létrehozunk egy utolso_x nevű változót aminek az első pont x koordinátája értéket adunk és ez később azt jelzi, hogy hol fejeztük be az utolsó szakaszt az 50-ből
    utolso_y = p0_y #létrehozunk egy utolso_y nevű változót aminek az első pont y koordinátája értéket adunk és ez később azt jelzi, hogy hol fejeztük be az utolsó szakaszt az 50-ből
    for i in range(1, 51): #fusson le a kód 50-szer(50 részre osztjuk a görbét)
        szazalek = i / 50 #létrehozunk egy szazalek nevű változót aminek az i/50(0-1) értéket adunk és ez később azt jelzi, hogy hán százaléka van meg az útnak
        tx = (1-szazalek)**3 * p0_x + 3*(1-szazalek)**2 * szazalek * p1_x + 3*(1-szazalek) * szazalek**2 * p2_x + szazalek**3 * p3_x
        ty = (1-szazalek)**3 * p0_y + 3*(1-szazalek)**2 * szazalek * p1_y + 3*(1-szazalek) * szazalek**2 * p2_y + szazalek**3 * p3_y
        bezier_hossz += sqrt((tx - utolso_x)**2 + (ty - utolso_y)**2)
        utolso_x, utolso_y = tx, ty
    db.reset()
    while True:
        megtett_ut_mm = db.distance()
        t = megtett_ut_mm / bezier_hossz
        if t >= 1.0:
            break
        tx = (1-t)**3 * p0_x + 3*(1-t)**2 * t * p1_x + 3*(1-t) * t**2 * p2_x + t**3 * p3_x
        ty = (1-t)**3 * p0_y + 3*(1-t)**2 * t * p1_y + 3*(1-t) * t**2 * p2_y + t**3 * p3_y
        t_elore = min(t + 0.05, 1.0) 
        nx = (1-t_elore)**3 * p0_x + 3*(1-t_elore)**2 * t_elore * p1_x + 3*(1-t_elore) * t_elore**2 * p2_x + t_elore**3 * p3_x
        ny = (1-t_elore)**3 * p0_y + 3*(1-t_elore)**2 * t_elore * p1_y + 3*(1-t_elore) * t_elore**2 * p2_y + t_elore**3 * p3_y
        
        dx = nx - tx
        dy = ny - ty
        
        elvart_szog = -degrees(atan2(dx, dy))
        aktualis_szog = -hub.imu.heading()
        
        szog_elteres = elvart_szog - aktualis_szog
        szog_elteres = (szog_elteres + 180) % 360 - 180
        
        kanyar_sebesseg = szog_elteres * 4.0 
        kanyar_sebesseg = max(min(kanyar_sebesseg, 65), -65)
        db.drive(speed=sebesseg, turn_rate=kanyar_sebesseg)
        await wait(10)
    db.stop()
    irany = hub.imu.heading() # Bézier után frissítjük az irányt

hub.system.set_stop_button(Button.BLUETOOTH) #beállítjuk a bluetooth gombot stop gombnak
hub.display.number(1) #az agy írja ki az 1-es számot
voltage = hub.battery.voltage() #létrehozunk egy voltage nevű változót aminek az agy töltöttségi szintjét adjuk értéknek és ez később azt jelzi, hogy mennyire van feltöltve a robot
print(voltage) #az agy írja ki a voltage-ot

async def futas_0(): #létrehozunk egy futas_0 nevü függvényt, azért async, hogy közben más mozgás is le tudjon futni
    hub.imu.reset_heading(0) #a gyro értékét 0-ra állítjuk
    bal.reset_angle(0) #a bal szögét 0-ra állítjuk
    jobb.reset_angle(0) #a jobb szögét 0-ra állítjuk
    await wait(200) #várj 200 millimásodpercet
 
async def futas_1(): #létrehozunk egy futas_1 nevü függvényt, azért async, hogy közben más mozgás is le tudjon futni
    hub.imu.reset_heading(0) #a gyro értékét 0-ra állítjuk
    bal.reset_angle(0) #a bal szögét 0-ra állítjuk
    jobb.reset_angle(0) #a jobb szögét 0-ra állítjuk
    await wait(200) #várj 200 millimásodpercet
    
async def futas_2(): #létrehozunk egy futas_2 nevü függvényt, azért async, hogy közben más mozgás is le tudjon futni
    hub.imu.reset_heading(0) #a gyro értékét 0-ra állítjuk
    bal.reset_angle(0) #a bal szögét 0-ra állítjuk
    jobb.reset_angle(0) #a jobb szögét 0-ra állítjuk
    await wait(200) #várj 200 millimásodpercet
 
async def futas_3(): #létrehozunk egy futas_3 nevü függvényt, azért async, hogy közben más mozgás is le tudjon futni
    hub.imu.reset_heading(0) #a gyro értékét 0-ra állítjuk
    bal.reset_angle(0) #a bal szögét 0-ra állítjuk
    jobb.reset_angle(0) #a jobb szögét 0-ra állítjuk
    await wait(200) #várj 200 millimásodpercet

async def futas_4(): #létrehozunk egy futas_4 nevü függvényt, azért async, hogy közben más mozgás is le tudjon futni
    hub.imu.reset_heading(0) #a gyro értékét 0-ra állítjuk
    bal.reset_angle(0) #a bal szögét 0-ra állítjuk
    jobb.reset_angle(0) #a jobb szögét 0-ra állítjuk
    await wait(200) #várj 200 millimásodpercet

async def futas_5(): #létrehozunk egy futas_5 nevü függvényt, azért async, hogy közben más mozgás is le tudjon futni
    hub.imu.reset_heading(0) #a gyro értékét 0-ra állítjuk
    bal.reset_angle(0) #a bal szögét 0-ra állítjuk
    jobb.reset_angle(0) #a jobb szögét 0-ra állítjuk
    await wait(200) #várj 200 millimásodpercet
 
futas = 0 #létrehozunk egy futas nevű változót aminek 0 értéket adunk és ez később azt jelzi, hogy melyik futásnál tart a robot
futasok = [futas_0, futas_1, futas_2, futas_3, futas_4, futas_5] #létrehozunk egy futasok nevű tömböt aminek futas_0, futas_1, futas_2, futas_3, futas_4, futas_5 értéket adunk és ez később azt jelzi, hogy melyik futások vannak
max_futas = len(futasok) #létrehozunk egy max_futas nevű változót aminek a futasok tömb nagyságát adjuk értéknek és ez később azt jelzi, hogy hány futásunk van és az 5. futásról a 0-ra menjen
 
while True: #elindítunk egy ciklust ami addig fut ameddig le nem állítjuk
    hub.display.number(futas + 1) #az agy írja ki a futas+1 számot
    megnyomva = [] #létrehozunk egy megnyomva nevű tömböt aminek üres értéket adunk és ez később azt jelzi, hogy melyik gombok vannak megnyomva
    while not any(megnyomva): #amig nincs semmi a megnyomva tömbben
        megnyomva = hub.buttons.pressed() #a megnyomott gomb legyen a megnyomva tömmben 
        wait(10) #várj 10 millimásodpercet
    
    lenyomott = StopWatch() #létrehozunk egy lenyomott nevű stoppert és ez később azt jelzi, hogy mennyi ideig nyomjuk meg a gombot
    rezgett = False #létrehozunk egy rezgett nevű változót aminek False értéket adunk meg és ez később azt jelzi, hogy rezgett e a feltét
    while hub.buttons.pressed(): #amig levan nyomva egy gomb
        if lenyomott.time() > 500: #ha a lenyomott stopper átlépte a fél másodpercet
            rezgett = True #a rezgett változót True-ra állítjuk
            feltet_bal.run_angle(900, 45, wait=False) #a feltet_bal motoron lefutattják a speedet és anglet
            feltet_jobb.run_angle(900, 45, wait=False)  #a feltet_jobb motoron lefutattják a speedet és anglet
            while not feltet_bal.done(): #amig a feltet_bal nincs kész(nem futott le a mozgás)
                wait(10) #várj 10 millimásodpercet
            feltet_bal.run_angle(900, -45, wait=False) #a feltet_bal motoron lefutattják a speedet és anglet
            feltet_jobb.run_angle(900, -45, wait=False) #a feltet_jobb motoron lefutattják a speedet és anglet
            while not feltet_bal.done(): #amig a feltet_jobb nincs kész(nem futott le a mozgás)
                wait(10) #várj 10 millimásodpercet
        wait(10) #várj 10 millimásodpercet
    
    if rezgett == True: #ha a rezgett változó True
        wait(200) #akkor várj 200 millimásodpercet
        continue #ugorjon a következő részre
    if Button.RIGHT in megnyomva: #ha a jobb gomb benne van a megnyomva tömbben
        futas = (futas + 1) % max_futas #akkor a futás legyen egyenlő futás + 1 maradéka a maxfutassal
    if Button.LEFT in megnyomva: #ha a bal gomb benne van a megnyomva tömbben
        futas = (futas - 1) % max_futas #akkor a futás legyen egyenlő futás - 1 maradéka a maxfutassal
    if Button.CENTER in megnyomva: #ha a középső gomb benne van a megnyomva tömbben
        irany = 0 #az irányt beállítjuk 0-ra(reseteljük)
        hub.imu.reset_heading(0) #a gyro értékét 0-ra állítjuk
        while Button.CENTER in hub.buttons.pressed(): #amig a középső gomb meg van nyomva
            wait(10) #várj 10 millimásodpercet
        try: #próbáld meg lefutatni a következő kódrészletet, ha bármikor ki lép a kódból(systemexit) menj az exceptre
            hub.system.set_stop_button(Button.CENTER) #beállítjuk a középső gombot stop gombra
            if elindult_timer == False and futas == 0: #ha az elindult:timer False és a futás 0 van
                meccs_ora = StopWatch() #létrehozunk egy meccs_ora nevű stoppert és ez később azt jelzi, hogy mennyi ideje fut a robot
                meccs_ora.reset() #akkor lenullázza a stoppert
                meccs_ora.resume() #akkor elindítja a stoppert
                elindult_timer = True #akkor az elindult_timer legyen True
            run_task(futasok[futas]()) #futassa le a futásoktömb futasadik elemét 
            futas = (futas + 1) % max_futas #a futás legyen egyenlő futás + 1 maradéka a maxfutassal
        except SystemExit: #ha volt systemexit(kilépett) a tryban jöjjön ide
            while Button.CENTER in hub.buttons.pressed(): #amig a középső gomb meg van nyomva
                wait(10) #várj 10 millimásodpercet
            bal.stop() #álljon le a bal motor
            jobb.stop() #álljon le a jobb motor
            feltet_bal.stop() #álljon le a feltet_bal motor
            feltet_jobb.stop() #álljon le a feltet_jobb motor
        hub.system.set_stop_button(Button.BLUETOOTH) #beállítjuk a bluetooth gombot stop gombra
    wait(10) #várj 10 millimásodpercet

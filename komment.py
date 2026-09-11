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
 
async def egyenes(e_tavolsag, e_legkisebb_sebesseg=40, e_gyorsitas=40, e_korekcio=0.01, e_legnagyobb_sebesseg = 700, e_lassitas=80, timeout = None): #létrehozunk egy egyenes nevü függvényt, paramétereket adunk meg amit használni fogunk a függvényben, az e_tavolsagot, e_lassitast és a e_gyorsitast mm-be adjuk meg, az alap értékek csak átlagban működnek, azért async, hogy közben más mozgás is le tudjon futni
    if timeout != None: #ha a timeoutnak van értéke
        timeout_watch = StopWatch() #akkor létrehoz egy stoppert
        timeout_watch.reset() #akkor lenullázza a stoppert
        timeout_watch.resume() #akkor elindítja a stoppert
    global irany #engedélyezzük a függvénynek az irany változó használatát a függvényen belül
    bal.reset_angle(0) #a bal szögét 0-ra állítjuk
    jobb.reset_angle(0) #a jobb szögét 0-ra állítjuk
    e_tavolsag = e_tavolsag / 0.489 #a mm-ben megadott e_tavolsagot átváltjuk motor fokokra
    e_gyorsitas /= 0.489 #a mm-ben megadott e_gyorsítástt átváltjuk motor fokokra
    e_lassitas /= 0.489 #a mm-ben megadott e_lassítást átváltjuk motor fokokra
    while True: #elindítunk egy ciklust ami addig fut ameddig le nem állítjuk
        if timeout != None and timeout_watch.time() >= timeout: #ha van a timeoutnak értéke és a stopper átlépte ezt az értéket
            break #akkor lépjen ki a ciklusból           
        e_megtett_tavolsag = (bal.angle()+jobb.angle()) / 2 #létrehozunk egy megtett_tavolsag nevű változót aminek jobb és bal motor szögének az átlagát adjuk értéknek és ez később azt jelzi, hogy mennyit haladt a robot
        e_hatralevo_tavolsag = e_tavolsag - e_megtett_tavolsag #létrehozunk egy hatralevo_tavolsag nevű változót aminek tavolsag-megtett_tavolsag értéket adunk és ez később azt jelzi, hogy mennyi távolság van hátra
        e_jelzo = e_hatralevo_tavolsag/abs(e_hatralevo_tavolsag) #létrehozunk egy jelzo nevű változót aminek hatralevo_tavolsag és annak az abszolut értékje osztva értéket adunk és ez később azt jelzi, hogy előre vagy hátra kell mennie a robotnak
        e_megtett_tavolsag = abs(e_megtett_tavolsag) #az e_megtett_tavolsag abszolut értéke legyen az e_megtett_tavolsag, azért kell hogy pozitiv legyen és a jelző már eltárolta, hogy negativ vagy pozitiv és később pozitivan számolunk vele
        e_hatralevo_tavolsag = abs(e_hatralevo_tavolsag) #az e_hatralevo_tavolsag abszolut értéke legyen az e_hatralevo_tavolsag, azért kell hogy pozitiv legyen és a jelző már eltárolta, hogy negativ vagy pozitiv és később pozitivan számolunk vele
        if e_hatralevo_tavolsag < 3: #ha e_hatralevo_tavolsag kevesebb, mint 3 motorfok(ha már elég közel van a célhoz)
            break #akkor lépjen ki a ciklusból
        if e_hatralevo_tavolsag < e_lassitas : #ha az e_hatralevo_tavolsag kevesebb, mint az e_lassitas(ha már a lassításba van)
            e_ratio = e_hatralevo_tavolsag / e_lassitas #akkor létrehozunk egy ratio nevű változót aminek hatralevo_tavolsag / lassitas értéket adunk és ez később azt jelzi, hogy milyen gyorsan és melyen mértékbe lassítson 
            e_mostani_sebesseg = max(e_ratio * e_legnagyobb_sebesseg, e_legkisebb_sebesseg) * e_jelzo #akkor létrehozunk egy mostani_sebesseg nevű változót aminek a nagyobb értéket adunk jelzo * a kettő közül ratio*legnagyobb_sebesseg vagy legkisebb_sebesseg értéket adunk és ez később azt jelzi, hogy mennyire gyorsan menjen előre
        elif e_megtett_tavolsag < e_gyorsitas: #ha nem az előző és a megtett_tavolsag kisebb, mint a gyorsitas(a gyorsitas szakaszban van)
            e_ratio = e_megtett_tavolsag / e_gyorsitas #akkor létrehozunk egy ratio nevű változót aminek megtett_tavolsag / gyorsitas értéket adunk és ez később azt jelzi, hogy hogy milyen gyorsan és melyen mértékbe gyorsitson
            e_mostani_sebesseg = max(e_ratio * e_legnagyobb_sebesseg, e_legkisebb_sebesseg) * e_jelzo #akkor létrehozunk egy mostani_sebesseg nevű változót aminek a nagyobb értéket adunk jelzo * a kettő közül ratio*legnagyobb_sebesseg vagy legkisebb_sebesseg értéket adunk és ez később azt jelzi, hogy mennyire gyorsan menjen előre
        else: #ha semelyik előző
            e_mostani_sebesseg = e_legnagyobb_sebesseg * e_jelzo #akkor létrehozunk egy mostani_sebesseg nevű változót aminek jelzo * legnagyobb_sebesseg értéket adunk és ez később azt jelzi, hogy mennyire gyorsan menjen előre
        e_iranyelteres = (irany - hub.imu.heading()) * e_korekcio #létrehozunk egy iranyelteres nevű változót aminek a (irany - gyro értéke) * korekcio értéket adunk és ez később azt jelzi, hogy mennyit tévedett a robot
        e_korekciomertek = e_mostani_sebesseg * e_iranyelteres * e_jelzo #létrehozunk egy korekcio_mertek nevű változót aminek a mostani_sebesseg * iranyelteres * jelzo értéket adunk és ez később azt jelzi, hogy milyen gyorsan és milyen kis mértékekben korigáljon
        bal.run (e_mostani_sebesseg - e_korekciomertek) #a bal motoron lefutattják a mostani_sebesseg - korkciomertek
        jobb.run(e_mostani_sebesseg + e_korekciomertek) #a jobb motoron lefutattják a mostani_sebesseg + korkciomertek
        await wait(10) #várj 10 millimásodpercet
    bal.stop() #álljon le a bal motor
    jobb.stop() #álljon le a jobb motor
    
 
async def kanyarodas(k_fok, k_legnagyobb_sebesseg=360, k_lassitas=80, k_legkisebb_sebesseg=50, timeout = None): #létrehozunk egy kanyarodas nevü függvényt, paramétereket adunk meg amit használni fogunk a függvényben, a fokot és e_lassitast motorfokokban-be adjuk meg, az alap értékek csak átlagban működnek, azért async, hogy közben más mozgás is le tudjon futni
    if timeout != None: #ha a timeoutnak van értéke
        timeout_watch = StopWatch() #akkor létrehoz egy stoppert
        timeout_watch.reset() #akkor lenullázza a stoppert
        timeout_watch.resume() #akkor elindítja a stoppert
    k_alap_fok = hub.imu.heading() #létrehozunk egy alap_fok nevű változót aminek a gyro értéket adunk és ez később azt jelzi, hogy ebből számoljuk a tévedést
    global irany #engedélyezzük a függvénynek az irany változó használatát a függvényen belül
    k_cel_fok = irany+k_fok #létrehozunk egy cel_fok nevű változót aminek az irany + fok értéket adunk és ez később azt jelzi, hogy ebből számoljuk a tévedést, mert ez adja meg az elméleti fokot
    irany = k_cel_fok #az irány változónak a cel_fok értéket adjuk meg
    k_cel_fok -= k_alap_fok #a cel_fok legyen egyenlő a cel_fok - alap_fok, ez így korigálja hibát
    while True: #elindítunk egy ciklust ami addig fut ameddig le nem állítjuk
        if timeout != None and timeout_watch.time() >= timeout: #ha van a timeoutnak értéke és a stopper átlépte ezt az értéket
            break #akkor lépjen ki a ciklusból  
        k_megtett_fokok = hub.imu.heading() - k_alap_fok #létrehozunk egy megtett_fokok nevű változót aminek a gyro - alap_fok értéket adunk és ez később azt jelzi, hogy ebből számoljuk a tévedést, mert ez lesz a megtett távolság
        k_hatralevo_fokok = k_cel_fok - k_megtett_fokok #létrehozunk egy hatralevo_fokok nevű változót aminek a cel_fok - megtett_fokok értéket adunk és ez később azt jelzi, hogy ebből számoljuk a tévedést, mert ez lesz a hátralévő távolság
        k_jelzo = k_hatralevo_fokok/abs(k_hatralevo_fokok) #létrehozunk egy jelzo nevű változót aminek hatralevo_fokok és annak az abszolut értékje osztva értéket adunk és ez később azt jelzi, hogy előre vagy hátra kell mennie a robotnak
        k_hatralevo_fokok = abs(k_hatralevo_fokok) #az e_hatralevo_fokok abszolut értéke legyen az e_hatralevo_fokok, azért kell hogy pozitiv legyen és a jelző már eltárolta, hogy negativ vagy pozitiv és később pozitivan számolunk vele
        if k_hatralevo_fokok <= 0.5: #ha e_hatralevo_fokok kevesebb, mint 3 motorfok(ha már elég közel van a célhoz)
            break #akkor lépjen ki a ciklusból
        if k_hatralevo_fokok < k_lassitas : #ha az hatralevo_fokok kevesebb, mint az lassitas(ha már a lassításba van)
            k_ratio = k_hatralevo_fokok / k_lassitas #akkor létrehozunk egy ratio nevű változót aminek hatralevp_fokok / lassitas értéket adunk és ez később azt jelzi, hogy hogy milyen gyorsan és melyen mértékbe lassítson
            k_mostani_sebesseg = max(k_ratio * k_legnagyobb_sebesseg, k_legkisebb_sebesseg) * k_jelzo #akkor létrehozunk egy mostani_sebesseg nevű változót aminek a nagyobb értéket adunk jelzo * a kettő közül ratio*legnagyobb_sebesseg vagy legkisebb_sebesseg értéket adunk és ez később azt jelzi, hogy mennyire gyorsan kanyarodjon
        else: #ha nem az előző 
            k_mostani_sebesseg = k_legnagyobb_sebesseg * k_jelzo #akkor létrehozunk egy mostani_sebesseg nevű változót aminek jelzo * legnagyobb_sebesseg értéket adunk és ez később azt jelzi, hogy mennyire gyorsan kanyarodjon
        bal.run(-k_mostani_sebesseg) #a bal motoron lefutattják a -mostani_sebesseg
        jobb.run(k_mostani_sebesseg) #a bal motoron lefutattják a mostani_sebesseg
        await wait(10) #várj 10 millimásodpercet
    bal.stop() #álljon le a bal motor
    jobb.stop() #álljon le a jobb motor


async def jobb_feltet(angle, speed=400, timeout=None): #létrehozunk egy jobb_feltet nevü függvényt, paramétereket adunk meg amit használni fogunk a függvényben, a fokot motorfokokban-be adjuk meg, az alap értékek csak átlagban működnek, azért async, hogy közben más mozgás is le tudjon futni
    if timeout != None: #ha a timeoutnak van értéke
        timeout_watch = StopWatch() #akkor létrehoz egy stoppert
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
        timeout_watch = StopWatch() #akkor létrehoz egy stoppert
        timeout_watch.reset() #akkor lenullázza a stoppert
        timeout_watch.resume() #akkor elindítja a stoppert
    feltet_bal.run_angle(speed, angle, wait=False) #a feltet_bal motoron lefutattják a speedet és anglet
    while not feltet_bal.done(): #amig a feltet_jobb nincs kész(nem futott le a mozgás)
        if timeout != None and timeout_watch.time() >= timeout: #ha van a timeoutnak értéke és a stopper átlépte ezt az értéket
            break #akkor lépjen ki a ciklusból 
        await wait(10) #várj 10 millimásodpercet
    feltet_bal.stop() #álljon le a feltét_bal motor

async def drivebase_bezier(p0_x, p0_y, p1_x, p1_y, p2_x, p2_y, p3_x, p3_y, alap_sebesseg=200):
    global irany # Globális változó frissítése
    görbe_hossz_mm = 0
    utolso_x, utolso_y = p0_x, p0_y
    for i in range(1, 51):
        temp_t = i / 50
        tx = (1-temp_t)**3 * p0_x + 3*(1-temp_t)**2 * temp_t * p1_x + 3*(1-temp_t) * temp_t**2 * p2_x + temp_t**3 * p3_x
        ty = (1-temp_t)**3 * p0_y + 3*(1-temp_t)**2 * temp_t * p1_y + 3*(1-temp_t) * temp_t**2 * p2_y + temp_t**3 * p3_y
        görbe_hossz_mm += sqrt((tx - utolso_x)**2 + (ty - utolso_y)**2)
        utolso_x, utolso_y = tx, ty
    db.reset()
    while True:
        megtett_ut_mm = db.distance()
        t = megtett_ut_mm / görbe_hossz_mm
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
        db.drive(speed=alap_sebesseg, turn_rate=kanyar_sebesseg)
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
    
    # Rezgés vizsgálat
    while hub.buttons.pressed():
        if lenyomott.time() > 500:
            rezgett = True
            feltet_bal.run_angle(900, 45, wait=False)
            feltet_jobb.run_angle(900, 45, wait=False)
            while not feltet_bal.done(): wait(10)
            
            feltet_bal.run_angle(900, -45, wait=False)
            feltet_jobb.run_angle(900, -45, wait=False)
            while not feltet_bal.done(): wait(10)
        wait(10)
    
    # Ha rezgett, ne lépjen be a menükezelésbe, csak folytassa a ciklust
    if rezgett:
        wait(200) # Várakozás az elengedésre
        continue 
    
    if Button.RIGHT in megnyomva: 
        futas = (futas + 1) % max_futas 
    if Button.LEFT in megnyomva: 
        futas = (futas - 1) % max_futas 
    if Button.CENTER in megnyomva: 
        irany = 0
        hub.imu.reset_heading(0) 
        
        while Button.CENTER in hub.buttons.pressed(): 
            wait(10)
            
        try: 
            hub.system.set_stop_button(Button.CENTER) 
            if elindult_timer == False and futas == 0:
                meccs_ora = StopWatch()
                meccs_ora.reset()
                meccs_ora.resume()
                elindult_timer = True
            run_task(futasok[futas]())
            futas = (futas + 1) % max_futas 
        except SystemExit: 
            while Button.CENTER in hub.buttons.pressed(): 
                wait(10)
            bal.stop() 
            jobb.stop() 
            feltet_bal.stop() 
            feltet_jobb.stop() 
        hub.system.set_stop_button(Button.BLUETOOTH) 
    wait(10)

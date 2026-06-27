from pybricks.hubs import * #beimportálja az agyat
from pybricks.pupdevices import * #beimportálja a motorokat
from pybricks.parameters import * #beimportálja a paramétereket
from pybricks.tools import * #beimportálja a toolsokat
from umath import * #beimportálja a matekot a bezier görbéhez
from pybricks.robotics import DriveBase #beimportálja a DriveBase-t a bezier görbéhez
 
hub = PrimeHub() #az agyat elnevezi hubnak
bal  = Motor(Port.B) #a motort ami a B portba van elnevezzük balnak és óra járásával megegyező irányba forog
jobb = Motor(Port.F, Direction.COUNTERCLOCKWISE) #a motort ami az F portba van elnevezzük balnak és óra járásával ellentétes irányba forog
feltet_bal = Motor(Port.A) #a feltét motort ami az E portba van elnevezzük feltet_balnak és a fogaskerék áttét 12, 12, 20
feltet_jobb = Motor(Port.E)  #a feltét motort ami az A portba van elnevezzük feltet_jobbnak és a fogaskerék áttét 12, 12, 20
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
elindult_timer = False #az elindult_timert Hamisra állítjuk, később erre lesz szükség a meccs mérésére
 
async def egyenes(e_tavolsag, e_legkisebb_sebesseg=40, e_gyorsitas=40, e_korekcio=0.01, e_legnagyobb_sebesseg = 700, e_lassitas=80, timeout = None): #létrehozunk egy egyenes nevü függvényt, paramétereket adunk meg amit használni fogunk a függvényben, az e_tavolsagot, e_lassitast és a e_gyorsitast mm-be adjuk meg, az alap értékek csak átlagban működnek, azért async, hogy közben más mozgás is le tudjon futni
    if timeout != None: #ha a timeoutnak van értéke
        timeout_watch = StopWatch() #létrehoz egy stoppert
        timeout_watch.reset() #lenullázza a stoppert
        timeout_watch.resume() #elindítja a stoppert
    global irany #engedélyezzük a függvénynek az irany változó használatát a függvényen belül
    bal.reset_angle(0) #a bal szögét 0-ra állítjuk
    jobb.reset_angle(0) #a jobb szögét 0-ra állítjuk
    e_tavolsag = e_tavolsag / 0.489 #a mm-ben megadott e_tavolsagot átváltjuk motor fokokra, a kerék kerületét elosztjuk 360nal
    e_gyorsitas /= 0.489 #a mm-ben megadott e_gyorsítástt átváltjuk motor fokokra, a kerék kerületét elosztjuk 360nal
    e_lassitas /= 0.489 #a mm-ben megadott e_lassítást átváltjuk motor fokokra, a kerék kerületét elosztjuk 360nal
    while True: #elindítunk egy ciklust ami addig fut ameddig le nem állítjuk
        if timeout != None and timeout_watch.time() >= timeout: #ha van a timeoutnak értéke és a stopper átlépte ezt az értéket
            break #akkor lépjen ki a ciklusból           
        e_megtett_tavolsag = (bal.angle()+jobb.angle()) / 2 #a bal és jobb szöget összeadjuk és elosztjuk 2-vel, kijön az átlagos megtetttávolság ami az e_megtett_tavolsag
        e_hatralevo_tavolsag = e_tavolsag - e_megtett_tavolsag #a tavolsagból kivonjuk az e_megtett_tavolsagot így kijön a e_hatralevo_tavolsag
        e_jelzo = e_hatralevo_tavolsag/abs(e_hatralevo_tavolsag) #az e_hatralevo_tavolsagot elosztjuk az e_hatralevo_tavolsag abszolutértékével, ha az e_hatralevo_tavolsag pozitiv akkor 1, ha negatív akkor -1 lesz a e_jelző értéke(pl 10/10=1 -10/10=-1)
        e_megtett_tavolsag = abs(e_megtett_tavolsag) #az e_megtett_tavolsag abszolut értéke legyen az e_megtett_tavolsag, azért kell hogy pozitiv legyen és a jelző már eltárolta, hogy negativ vagy pozitiv
        e_hatralevo_tavolsag = abs(e_hatralevo_tavolsag) #az e_hatralevo_tavolsag abszolut értéke legyen az e_hatralevo_tavolsag, azért kell hogy pozitiv legyen és a jelző már eltárolta, hogy negativ vagy pozitiv
        if e_hatralevo_tavolsag < 3: #ha e_hatralevo_tavolsag kevesebb, mint 3 motorfok
            break #akkor lépjen ki a ciklusból
        if e_hatralevo_tavolsag < e_lassitas : #ha az e_hatralevo_tavolsag kevesebb, mint az e_lassitas(ha már a lassításba van)
            e_ratio = e_hatralevo_tavolsag / e_lassitas #akkor a lassitas ratioja az e_hatralevo_tavolsag/e_lassitas, ez legyen az e_ratio ez a szám 0-1ig van
            e_mostani_sebesseg = max(e_ratio * e_legnagyobb_sebesseg, e_legkisebb_sebesseg) * e_jelzo #az e_mostani_sebesseg legyen a max(vagyis a nagyobb) az e_legkisebb_sebesseg(hogy ne menjen lassabban) vagy a e_ratio*e_legnagyobb_sebesseg és ez megszroozva az e_jelzo-vel ami 1 vagy -1, hogy pozitiv vagy negativ legyen
        elif e_megtett_tavolsag < e_gyorsitas: 
            e_ratio = e_megtett_tavolsag / e_gyorsitas 
            e_mostani_sebesseg = max(e_ratio * e_legnagyobb_sebesseg, e_legkisebb_sebesseg) * e_jelzo 
        else: 
            e_mostani_sebesseg = e_legnagyobb_sebesseg * e_jelzo 
        e_iranyelteres = (irany - hub.imu.heading()) * e_korekcio 
        e_korekciomertek = e_mostani_sebesseg * e_iranyelteres * e_jelzo 
        bal.run (e_mostani_sebesseg - e_korekciomertek) 
        jobb.run(e_mostani_sebesseg + e_korekciomertek) 
        await wait(10)
    bal.stop() 
    jobb.stop()
    
 
async def kanyarodas(k_fok, k_legnagyobb_sebesseg=360, k_lassitas=80, k_legkisebb_sebesseg=50, timeout = None): 
    if timeout != None:
        timeout_watch = StopWatch()
        timeout_watch.reset()
        timeout_watch.resume()
    k_alap_fok = hub.imu.heading() 
    global irany 
    k_cel_fok = irany+k_fok 
    irany = k_cel_fok 
    k_cel_fok -= k_alap_fok 
    while True: 
        if timeout != None and timeout_watch.time() >= timeout: break
        k_megtett_fokok = hub.imu.heading() - k_alap_fok 
        k_hatralevo_fokok = k_cel_fok - k_megtett_fokok 
        k_jelzo = k_hatralevo_fokok/abs(k_hatralevo_fokok) 
        k_hatralevo_fokok = abs(k_hatralevo_fokok) 
        if k_hatralevo_fokok <= 0.5: 
            break      
        if k_hatralevo_fokok < k_lassitas : 
            k_ratio = k_hatralevo_fokok / k_lassitas 
            k_mostani_sebesseg = max(k_ratio * k_legnagyobb_sebesseg, k_legkisebb_sebesseg) * k_jelzo 
        else: 
            k_mostani_sebesseg = k_legnagyobb_sebesseg * k_jelzo 
        bal.run(-k_mostani_sebesseg) 
        jobb.run(k_mostani_sebesseg) 
        await wait(10)
    bal.stop() 
    jobb.stop() 
    await wait(100)

async def jobb_feltet(angle, speed=400, timeout=None): 
    if timeout != None:
        timeout_watch = StopWatch()
        timeout_watch.reset()
        timeout_watch.resume()
    
    # Indítsd a relatív mozgást
    feltet_jobb.run_angle(speed, angle, wait=False) 
    
    # Addig várj, amíg a motor nem jelzi, hogy végzett (done)
    while not feltet_jobb.done():
        if timeout != None and timeout_watch.time() >= timeout: 
            break
        await wait(10)
    feltet_jobb.stop() 
 
async def bal_feltet(angle, speed=400, timeout=None): 
    if timeout != None:
        timeout_watch = StopWatch()
        timeout_watch.reset()
        timeout_watch.resume()
    
    # Indítsd a relatív mozgást
    feltet_bal.run_angle(speed, angle, wait=False) 
    
    # Addig várj, amíg a motor nem jelzi, hogy végzett (done)
    while not feltet_bal.done():
        if timeout != None and timeout_watch.time() >= timeout: 
            break
        await wait(10)
    feltet_bal.stop()  

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

hub.system.set_stop_button(Button.BLUETOOTH)
hub.display.number(1)
voltage = hub.battery.voltage()
print(voltage)
 
async def futas_1(): 
    hub.imu.reset_heading(0) 
    feltet_bal.reset_angle(0)
    feltet_jobb.reset_angle(0)
    await wait(200)
    
async def futas_2():
    hub.imu.reset_heading(0) 
    feltet_bal.reset_angle(0)
    feltet_jobb.reset_angle(0)
    await wait(200)
 
async def futas_3(): 
    hub.imu.reset_heading(0) 
    feltet_bal.reset_angle(0)
    feltet_jobb.reset_angle(0)
    await wait(200)

async def futas_4(): 
    hub.imu.reset_heading(0) 
    feltet_bal.reset_angle(0)
    feltet_jobb.reset_angle(0)
    await wait(200)

futas = 0 
futasok = [futas_1, futas_2, futas_3, futas_4] 
max_futas = len(futasok) 
 
while True: 
    hub.display.number(futas + 1) 
    megnyomva = [] 
    while not any(megnyomva): 
        megnyomva = hub.buttons.pressed()  
        wait(10)
    
    lenyomott = StopWatch() 
    rezgett = False # Rezgés jelző
    
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

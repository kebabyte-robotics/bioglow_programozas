from pybricks.hubs import * 
from pybricks.pupdevices import * 
from pybricks.parameters import *
from pybricks.tools import * 
from umath import * 
from pybricks.robotics import DriveBase 
 
hub = PrimeHub()
bal  = Motor(Port.B) 
jobb = Motor(Port.F, Direction.COUNTERCLOCKWISE) 
feltet_bal = Motor(Port.A) 
feltet_jobb = Motor(Port.E)  
feltet_bal.control.limits(1000, 10000) 
feltet_jobb.control.limits(1000, 6500) 
bal.control.limits(2000, 5500) 
jobb.control.limits(2000, 5000) 
db = DriveBase(bal, jobb, wheel_diameter=56, axle_track=110) 
db.use_gyro(True) 
while not hub.imu.ready(): 
    hub.display.char("x") 
 
hub.imu.reset_heading(0) 
irany = 0 
elindult_timer = False 
 
async def egyenes(tavolsag, legkisebb_sebesseg=40, gyorsitas=40, korekcio=0.01, legnagyobb_sebesseg = 700, lassitas=80, timeout = None):
    if timeout != None:
        timeout_watch = StopWatch()
        timeout_watch.reset()
        timeout_watch.resume()
    global irany
    bal.reset_angle(0)
    jobb.reset_angle(0)
    tavolsag = tavolsag / 0.489
    gyorsitas /= 0.489
    lassitas /= 0.489
    while True:
        if timeout != None and timeout_watch.time() >= timeout:
            break
        megtett_tavolsag = -(bal.angle()+jobb.angle()) / 2
        hatralevo_tavolsag = tavolsag - megtett_tavolsag
        jelzo = hatralevo_tavolsag/abs(hatralevo_tavolsag)
        megtett_tavolsag = abs(megtett_tavolsag)
        hatralevo_tavolsag = abs(hatralevo_tavolsag)
        if hatralevo_tavolsag < 3:
            break
        if hatralevo_tavolsag < lassitas :
            ratio = hatralevo_tavolsag / lassitas
            mostani_sebesseg = max(ratio * legnagyobb_sebesseg, legkisebb_sebesseg) * jelzo
        elif megtett_tavolsag < gyorsitas:
            ratio = megtett_tavolsag / gyorsitas
            mostani_sebesseg = max(ratio * legnagyobb_sebesseg, legkisebb_sebesseg) * jelzo
        else:
            mostani_sebesseg = legnagyobb_sebesseg * jelzo
        iranyelteres = (irany - hub.imu.heading()) * korekcio
        korekciomertek = mostani_sebesseg * iranyelteres * jelzo
        bal.run ((mostani_sebesseg + korekciomertek)*-1)
        jobb.run((mostani_sebesseg - korekciomertek)*-1)
        await wait(10)
    bal.stop()
    jobb.stop()
    
 
async def kanyarodas(fok, legnagyobb_sebesseg=360, lassitas=80, legkisebb_sebesseg=50, timeout = None):
    if timeout != None:
        timeout_watch = StopWatch()
        timeout_watch.reset()
        timeout_watch.resume()
    alap_fok = hub.imu.heading()
    global irany
    cel_fok = irany+fok
    irany = cel_fok
    cel_fok -= alap_fok
    while True:
        if timeout != None and timeout_watch.time() >= timeout:
            break
        megtett_fokok = hub.imu.heading() - alap_fok
        hatralevo_fokok = cel_fok - megtett_fokok
        jelzo = hatralevo_fokok/abs(hatralevo_fokok)
        hatralevo_fokok = abs(hatralevo_fokok)
        if hatralevo_fokok <= 0.5:
            break
        if hatralevo_fokok < lassitas :
            ratio = hatralevo_fokok / lassitas
            mostani_sebesseg = max(ratio * legnagyobb_sebesseg, legkisebb_sebesseg) * jelzo
        else:
            mostani_sebesseg = legnagyobb_sebesseg * jelzo
        bal.run(-mostani_sebesseg)
        jobb.run(mostani_sebesseg)
        await wait(10)
    bal.stop()
    jobb.stop()


async def jobb_feltet(angle, speed=400, timeout=None):
    if timeout != None:
        timeout_watch = StopWatch()
        timeout_watch.reset()
        timeout_watch.resume()
    feltet_jobb.run_angle(speed, angle, wait=False)
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
    feltet_bal.run_angle(speed, angle, wait=False)
    while not feltet_bal.done():
        if timeout != None and timeout_watch.time() >= timeout:
            break
        await wait(10)
    feltet_bal.stop()

async def bezier_gorbe(p0_x, p0_y, p1_x, p1_y, p2_x, p2_y, p3_x, p3_y, sebesseg=200, megforditva=True):
    global irany
    if megforditva == True:
        p0_x, p3_x = p3_x, p0_x
        p0_y, p3_y = p3_y, p0_y
        p1_x, p2_x = p2_x, p1_x
        p1_y, p2_y = p2_y, p1_y
    bezier_hossz = 0
    utolso_x = p0_x
    utolso_y = p0_y
    for i in range(1, 51):
        szazalek = i / 50
        szakasz_vege_x = (1-szazalek)**3 * p0_x + 3*(1-szazalek)**2 * szazalek * p1_x + 3*(1-szazalek) * szazalek**2 * p2_x + szazalek**3 * p3_x
        szakasz_vege_y = (1-szazalek)**3 * p0_y + 3*(1-szazalek)**2 * szazalek * p1_y + 3*(1-szazalek) * szazalek**2 * p2_y + szazalek**3 * p3_y
        bezier_hossz += sqrt((szakasz_vege_x - utolso_x)**2 + (szakasz_vege_y - utolso_y)**2)
        utolso_x = szakasz_vege_x
        utolso_y = szakasz_vege_y
    db.reset()
    while True:
        megtett_ut = db.distance()
        megtett_arany = abs(megtett_ut) / bezier_hossz
        if megtett_arany >= 1.0:
            break
        szakasz_vege_x = (1-megtett_arany)**3 * p0_x + 3*(1-megtett_arany)**2 * megtett_arany * p1_x + 3*(1-megtett_arany) * megtett_arany**2 * p2_x + megtett_arany**3 * p3_x
        szakasz_vege_y = (1-megtett_arany)**3 * p0_y + 3*(1-megtett_arany)**2 * megtett_arany * p1_y + 3*(1-megtett_arany) * megtett_arany**2 * p2_y + megtett_arany**3 * p3_y
        kovetkezo_arany = min(megtett_arany + 0.05, 1.0)
        kovetkezo_x = (1-kovetkezo_arany)**3 * p0_x + 3*(1-kovetkezo_arany)**2 * kovetkezo_arany * p1_x + 3*(1-kovetkezo_arany) * kovetkezo_arany**2 * p2_x + kovetkezo_arany**3 * p3_x
        kovetkezo_y = (1-kovetkezo_arany)**3 * p0_y + 3*(1-kovetkezo_arany)**2 * kovetkezo_arany * p1_y + 3*(1-kovetkezo_arany) * kovetkezo_arany**2 * p2_y + kovetkezo_arany**3 * p3_y
        tavolsag_x = kovetkezo_x - szakasz_vege_x
        tavolsag_y = kovetkezo_y - szakasz_vege_y
        if megforditva == True:
            elvart_szog = -degrees(atan2(-tavolsag_x, -tavolsag_y))
        else:
            elvart_szog = -degrees(atan2(tavolsag_x, tavolsag_y))
        aktualis_szog = -hub.imu.heading()
        szog_elteres = elvart_szog - aktualis_szog
        szog_elteres = (szog_elteres + 180) % 360 - 180
        kanyar_sebesseg = szog_elteres * 4.0
        kanyar_sebesseg = max(min(kanyar_sebesseg, 65), -65)
        if megforditva == True:
            aktualis_sebesseg = -sebesseg
        else:
            aktualis_sebesseg = sebesseg
        db.drive(speed=aktualis_sebesseg, turn_rate=kanyar_sebesseg)
        await wait(10)
    db.stop()
    irany = hub.imu.heading()

hub.system.set_stop_button(Button.BLUETOOTH)
hub.display.number(1)
voltage = hub.battery.voltage()
print(voltage)

async def futas_1():
    hub.imu.reset_heading(0)
    bal.reset_angle(0)
    jobb.reset_angle(0)
    await wait(200)
    await bezier_gorbe(0, 0, 100, 500, 200, 10, 300, 50, 200, True)

async def futas_2():
    hub.imu.reset_heading(0)
    bal.reset_angle(0)
    jobb.reset_angle(0)
    await wait(200)
 
async def futas_3():
    hub.imu.reset_heading(0)
    bal.reset_angle(0)
    jobb.reset_angle(0)
    await wait(200)

async def futas_4():
    hub.imu.reset_heading(0)
    bal.reset_angle(0)
    jobb.reset_angle(0)
    await wait(200)

async def futas_5():
    hub.imu.reset_heading(0)
    bal.reset_angle(0)
    jobb.reset_angle(0)
    await wait(200)

async def futas_6():
    hub.imu.reset_heading(0)
    bal.reset_angle(0)
    jobb.reset_angle(0)
    await wait(200)

futas = 0
futasok = [futas_1, futas_2, futas_3, futas_4, futas_5, futas_6]
max_futas = len(futasok)
 
while True:
    hub.display.number(futas + 1)
    megnyomva = []
    while not any(megnyomva):
        megnyomva = hub.buttons.pressed()
        wait(10)
    
    lenyomott = StopWatch()
    rezgett = False
    while hub.buttons.pressed():
        if lenyomott.time() > 500:
            rezgett = True
            feltet_bal.run_angle(900, 45, wait=False)
            feltet_jobb.run_angle(900, 45, wait=False) 
            while not feltet_bal.done():
                wait(10)
            feltet_bal.run_angle(900, -45, wait=False)
            feltet_jobb.run_angle(900, -45, wait=False)
            while not feltet_bal.done():
                wait(10)
        wait(10)
    
    if rezgett == True:
        wait(200)
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

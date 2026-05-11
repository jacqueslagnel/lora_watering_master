# README utilisateur

## But
Ce système permet de piloter un master LoRa et des nodes d'arrosage via MQTT, avec mise à jour OTA BLE à la demande.

## Commandes MQTT

### Commandes master
- `M;list` : liste des nodes connus
- `M;schedule;<seconds>` : règle la période de batterie automatique
- `M;ota;<seconds>` : redémarre le master en mode OTA BLE pendant une durée limitée

### Commandes node
- `N;<id>;ping`
- `N;<id>;bat`
- `N;<id>;status`
- `N;<id>;reboot`
- `N;<id>;ota`
- `N;<id>;led;on`
- `N;<id>;led;off`
- `N;<id>;period;<seconds>`
- `N;<id>;water;<seconds>`
- `N;<id>;abort`
- `N;<id>;wstatus`
- `N;<id>;valve;open`
- `N;<id>;valve;close`
- `N;<id>;text;<message>`

## Retours MQTT
Les messages de retour confirment l'état, le résultat des commandes, la batterie, l'état d'arrosage, les pulses et les litres.

Le broker ou le système superviseur horodate les messages à la réception.

## OTA BLE

### Master
1. Envoyer `M;ota;<seconds>`
2. Le master redémarre en mode BLE OTA
3. Faire la mise à jour avec nRF DFU
4. À la fin ou au timeout, le master redémarre normalement

### Slave
1. Envoyer `N;<id>;ota`
2. Le node redémarre en mode BLE OTA
3. Faire la mise à jour avec nRF DFU
4. À la fin ou au timeout, le node redémarre normalement

## Important
- Il n'y a pas de RTC/epoch embarqué dans le protocole
- Les timestamps doivent être gérés côté backend
- Le système est volontairement simplifié pour être plus robuste

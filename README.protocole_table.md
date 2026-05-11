## Tableau synthétique du protocole master

### Commandes master

| Commande MQTT | Action master | LoRa envoyée | Retour MQTT |
|---|---|---|---|
| `M;list` | liste les nodes connus | aucune | `M;list;...` |
| `M;schedule;<seconds>` | change la période batterie auto | aucune | `M;schedule;1;<seconds>` ou échec |
| `M;ota;<seconds>` | reboot en mode BLE OTA | aucune | `M;ota;1;<seconds>` ou échec |

### Commandes node

| Commande MQTT | Commande LoRa | Quand envoyée | Retour MQTT attendu |
|---|---|---|---|
| `N;<id>;ping` | `CMD_PING` | au prochain `HELLO` du node | `N;<id>;ping;1` ou échec |
| `N;<id>;bat` | `CMD_BATTERY` | au prochain `HELLO` | `N;<id>;bat;1;<mV>` ou échec |
| `N;<id>;status` | `CMD_STATUS` | au prochain `HELLO` | `N;<id>;status;1;...` ou échec |
| `N;<id>;text;<msg>` | `CMD_TEXT_MESSAGE` | au prochain `HELLO` | `N;<id>;text;1;...` ou échec |
| `N;<id>;reboot` | `CMD_REBOOT` | au prochain `HELLO` | `N;<id>;reboot;1` ou échec |
| `N;<id>;ota` | `CMD_OTA_MODE` | au prochain `HELLO` | `N;<id>;ota;1` ou échec |

### Commandes LED / période

| Commande MQTT | Commande LoRa | Quand envoyée | Retour MQTT attendu |
|---|---|---|---|
| `N;<id>;led;on` | `CMD_LED_ON` | au prochain `HELLO` | `N;<id>;led;1;on` ou échec |
| `N;<id>;led;off` | `CMD_LED_OFF` | au prochain `HELLO` | `N;<id>;led;1;off` ou échec |
| `N;<id>;period;<seconds>` | `CMD_SET_PERIOD` | au prochain `HELLO` | `N;<id>;period;1;<seconds>` ou échec |

### Commandes vanne / arrosage

| Commande MQTT | Commande LoRa | Quand envoyée | Retour MQTT attendu |
|---|---|---|---|
| `N;<id>;valve;open` | `CMD_VALVE_OPEN` | au prochain `HELLO` | `N;<id>;valve;1;open` ou échec |
| `N;<id>;valve;close` | `CMD_VALVE_CLOSE` | au prochain `HELLO` | `N;<id>;valve;1;close` ou échec |
| `N;<id>;water;<seconds>` | `CMD_WATER_TIME` | au prochain `HELLO` | `N;<id>;water;1;<valveOpen>;<wateringActive>;<durationSec>;<remainingSec>;<flowPulses>;<litres>` ou échec |
| `N;<id>;abort` | `CMD_WATER_ABORT` | au prochain `HELLO` | `N;<id>;abort;1;<valveOpen>;<wateringActive>;<durationSec>;<remainingSec>;<flowPulses>;<litres>` ou échec |
| `N;<id>;wstatus` | `CMD_WATER_STATUS` | au prochain `HELLO` | `N;<id>;wstatus;1;<valveOpen>;<wateringActive>;<durationSec>;<remainingSec>;<flowPulses>;<litres>` ou échec |

### Messages publiés spontanément par le master

| MQTT publié | Origine | Contenu |
|---|---|---|
| `H;...` | heartbeat périodique master | compteur heartbeat, batterie master, nombre de nodes online, liste des nodes |
| `LTE_CONNECTED;...` | connexion ou reconnexion MQTT LTE | opérateur, technologie radio (`NB-IoT` ou `Cat-M1`), champs signal modem |
| `J;...` | découverte / présence node | annonce de node vu |
| autres `N;...` | retour de commande node | succès ou échec avec payload décodé |

### Message LTE publié par le master

Au rétablissement d'une session MQTT LTE, le master publie :

```text
LTE_CONNECTED;<operator>;<rat>[;<signal_fields...>]
```

Exemples :

```text
LTE_CONNECTED;Bouygues;NB-IoT;...
LTE_CONNECTED;Orange;Cat-M1;...
```

Les champs signal dépendent du modem :

- BG77 : données `AT+QCSQ`
- SIM7080G : données `AT+CPSI?`

Il n'y a pas de message MQTT de déconnexion LTE dans le firmware actuel.

### Règle importante

Pour les commandes `N;<id>;...`, le flux est toujours :

`MQTT entrant -> mise en file -> prochain HELLO du node -> commande LoRa -> ACK LoRa -> MQTT sortant`

Donc une commande node n’est pas exécutée immédiatement à la réception MQTT, mais **à la prochaine fenêtre radio du node**.

### Remarques

- Il n’y a **pas de RTC/epoch embarqué** dans le protocole.
- Le backend doit horodater les messages à la réception.
- `flowPulses` est en **uint32_t**.
- `litres` est en **uint16_t**.

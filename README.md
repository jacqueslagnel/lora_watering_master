# Master / Slave LoRa Watering - Documentation minimale

## Vue d'ensemble

Le système est composé :

- d'un **master** RAK4631 avec LoRa + BG77 MQTT
- de **nodes/slaves** LoRa pour batterie, statut, vanne, arrosage et OTA BLE

Le backend horodate les messages MQTT à la réception.
Le protocole embarqué **n'embarque volontairement pas de RTC ni d'epoch**.

---

## Commandes MQTT master

Format général :

```text
M;<commande>[;<argument>]
```

### Commandes supportées

#### Passage du master en mode OTA BLE

```text
M;ota;<seconds>
```

Exemple :

```text
M;ota;300
```

Effet :

- le master accuse réception sur MQTT
- le master programme un reboot spécial OTA
- le master redémarre en mode BLE OTA
- advertising BLE pendant une fenêtre limitée
- retour au mode normal après timeout ou après mise à jour

#### Demande de liste des nodes connus

```text
M;list
```

Effet :

- le master publie la liste des nodes actuellement connus / en ligne

#### Programmation d'un scheduling master interne

```text
M;schedule;<seconds>
```

Exemple :

```text
M;schedule;300
```

Effet :

- met à jour une temporisation interne de scheduling côté master

---

## Commandes MQTT node

Format général :

```text
N;<node_id>;<commande>[;<argument>]
```

Exemples de `node_id` :

```text
N;1;...
N;2;...
N;3;...
```

### Commandes supportées

#### Ping

```text
N;<node_id>;ping
```

#### Batterie

```text
N;<node_id>;bat
```

#### Statut node

```text
N;<node_id>;status
```

#### Message texte

```text
N;<node_id>;text;<message>
```

Exemple :

```text
N;2;text;bonjour
```

#### Reboot node

```text
N;<node_id>;reboot
```

#### Changement de période de réveil / hello

```text
N;<node_id>;period;<seconds>
```

#### LED ON/OFF

```text
N;<node_id>;led;on
N;<node_id>;led;off
```

#### Ouverture / fermeture vanne

```text
N;<node_id>;valve;open
N;<node_id>;valve;close
```

#### Lancement arrosage temporisé

```text
N;<node_id>;water;<seconds>
```

Exemple :

```text
N;3;water;300
```

#### Arrêt arrosage

```text
N;<node_id>;abort
```

#### Statut arrosage

```text
N;<node_id>;wstatus
```

#### Passage du slave en mode OTA BLE

```text
N;<node_id>;ota
```

Effet :

- le master envoie une commande LoRa OTA au slave
- le slave accuse réception
- le slave redémarre en mode BLE OTA
- advertising BLE pendant une fenêtre limitée
- retour au mode normal après timeout ou après mise à jour

---

## Retours MQTT

Les formats ci-dessous sont les retours publiés par le master vers le backend.
Les timestamps absolus ne sont **pas** envoyés par l'embarqué.
Le backend doit horodater les messages à réception.

### Heartbeat master

```text
H;<counter>;<battery_mV>;<online_count>;<online_csv>
```

Exemple :

```text
H;12;3721;3;1,2,4
```

### Connexion LTE / MQTT rétablie

Quand le modem LTE vient d'établir ou de rétablir la session MQTT, le master publie :

```text
LTE_CONNECTED;<operator>;<rat>[;<signal_fields...>]
```

Ce message est publié par `main.cpp` lorsque `lte_modem.consumeJustConnected()` retourne vrai. Il est suivi d'une republication des présences des nodes connus.

Exemples possibles :

```text
LTE_CONNECTED;Bouygues;NB-IoT;<signal_fields...>
LTE_CONNECTED;Orange;Cat-M1;<signal_fields...>
```

Le champ `<rat>` indique la technologie radio utilisée :

- `NB-IoT`
- `Cat-M1`

Les champs suivants dépendent du modem actif :

- BG77 : valeurs issues de `AT+QCSQ`, séparées par des `;`
- SIM7080G : valeurs issues de `AT+CPSI?`, séparées par des `;`

Exemple logique :

```text
LTE_CONNECTED;Bouygues;NB-IoT;...
```

Il n'y a pas de message MQTT `LTE_DISCONNECTED` dans l'implémentation actuelle, car une perte LTE/MQTT empêche justement de publier vers le broker. Les pertes et reconnexions sont visibles sur la console série via les logs modem et les snapshots d'état.

### Retour commande master OTA

```text
M;ota;<ok>;<seconds>
```

Exemple :

```text
M;ota;1;300
```

### Retour commande master list

```text
M;list;<ok>;<count>;<csv>
```

Exemple :

```text
M;list;1;3;1,2,4
```

### Retour commande master schedule

```text
M;schedule;<ok>;<seconds>
```

Exemple :

```text
M;schedule;1;300
```

### Retour ping node

```text
N;<node_id>;ping;<ok>
```

### Retour batterie node

```text
N;<node_id>;bat;<ok>;<battery_mV>;<status>
```

### Retour statut node

```text
N;<node_id>;status;<ok>;<battery_mV>;<status_flags>;<uptime_sec>;<sleep_sec>
```

### Retour texte

```text
N;<node_id>;text;<ok>
```

### Retour reboot node

```text
N;<node_id>;reboot;<ok>
```

### Retour période node

```text
N;<node_id>;period;<ok>;<seconds>
```

### Retour LED node

```text
N;<node_id>;led;<ok>
```

### Retour vanne

```text
N;<node_id>;valve;<ok>;<valve_open>
```

### Retour arrosage lancé

```text
N;<node_id>;water;<ok>;<valve_open>;<watering_active>;<duration_sec>;<remaining_sec>;<flow_pulses>;<litres>
```

Exemple :

```text
N;3;water;1;1;1;300;300;0;0
```

### Retour arrêt arrosage

```text
N;<node_id>;abort;<ok>;<valve_open>;<watering_active>;<duration_sec>;<remaining_sec>;<flow_pulses>;<litres>
```

### Retour statut arrosage

```text
N;<node_id>;wstatus;<ok>;<valve_open>;<watering_active>;<duration_sec>;<remaining_sec>;<flow_pulses>;<litres>
```

Exemple :

```text
N;3;wstatus;1;1;1;300;234;66;1
```

### Retour OTA slave

```text
N;<node_id>;ota;<ok>
```

Exemple :

```text
N;2;ota;1
```

---

## Stratégie OTA master

### Mode normal

Au boot normal :

- BLE désactivé
- LoRa actif
- BG77 / MQTT actif
- fonctionnement nominal

### Entrée en OTA

Commande MQTT :

```text
M;ota;<seconds>
```

Séquence :

1. le master publie `M;ota;1;<seconds>`
2. le master enregistre un flag de boot OTA
3. le master redémarre
4. au boot, le flag OTA est détecté
5. le master démarre en **BLE OTA mode only**
6. le nom BLE annoncé est `MASTER_OTA`
7. une application DFU BLE peut pousser le `firmware.zip`
8. en cas de timeout OTA, reboot automatique en mode normal

### Remarque

La durée demandée peut être validée par la commande, mais l'implémentation effective peut utiliser une fenêtre OTA fixe selon la version du firmware.

---

## Stratégie OTA slave

### Mode normal

Au boot normal :

- BLE désactivé
- LoRa slave actif
- watering / flow / battery actifs selon la carte

### Entrée en OTA

Commande MQTT :

```text
N;<node_id>;ota
```

Séquence :

1. le backend envoie la commande MQTT au master
2. le master la convertit en commande LoRa `CMD_OTA_MODE`
3. le slave reçoit la commande et renvoie un ACK
4. le slave enregistre un flag de boot OTA
5. le slave redémarre
6. au boot, le flag OTA est détecté
7. le slave démarre en **BLE OTA mode only**
8. le nom BLE annoncé est typiquement :

```text
SLAVE_<node_id>_OTA
```

Exemple :

```text
SLAVE_2_OTA
```

9. après timeout OTA, reboot automatique en mode normal si aucune mise à jour n'a été faite

### Remarque carte

La logique OTA applicative est identique entre :

- RAK4631
- Heltec T114

mais chaque carte doit avoir **son bootloader OTA compatible propre**.

---

## Absence volontaire de RTC / epoch embarqué

Le système ne transporte volontairement **aucun epoch ni timestamp absolu embarqué**.

### Choix d'architecture

Les raisons sont :

- réduction de complexité
- suppression de la dépendance au temps LTE / modem
- suppression des problèmes de resync RTC
- suppression des boucles de retry RTC
- meilleure robustesse globale
- protocole LoRa plus simple

### Conséquence

Les payloads MQTT :

- ne contiennent pas d'epoch
- ne contiennent pas de date/heure absolue
- sont horodatés par le backend à réception

### Conséquence pratique

Le protocole se base uniquement sur :

- états
- durées
- compteurs
- pulses
- litres
- uptime local si nécessaire

et non sur une horloge temps réel partagée.

---

## Notes d'exploitation

- Les messages MQTT doivent être horodatés côté backend.
- Le mode OTA n'est pas permanent : il est activé à la demande puis le système revient en mode normal.
- `flow_pulses` est en `uint32_t`.
- `litres` reste en `uint16_t`.

---

## Documentation Doxygen

Le projet contient des commentaires Doxygen dans les fichiers applicatifs :

- `include/ble_ota.h`
- `include/watchdog_simple.h`
- `include/credentials.h`
- `include/lora_master.h`
- `include/bg77_mqtt.h`
- `include/sim7080_mqtt.h`
- `src/main.cpp`
- `src/ble_ota.cpp`
- `src/watchdog_simple.cpp`
- `src/lora_master.cpp`
- `src/bg77_mqtt.cpp`
- `src/sim7080_mqtt.cpp`

Les fichiers de support carte situés dans `rakwireless/` ne font pas partie de la documentation applicative.

### Installation

Sur macOS :

```bash
brew install doxygen
```

Optionnel, pour les graphes d’appel et diagrammes :

```bash
brew install graphviz
```

### Génération HTML

Depuis le dossier du projet :

```bash
cd /Users/jacques/Documents/PlatformIO/Projects/lora_p2p_master_slaves/pour_validations/RAK4631_watering_lora_p2p_master_ble_ota_dfu_noRTC_clean_fuites_multi-ops_dual_modem
doxygen Doxyfile
```

Puis ouvrir la documentation :

```bash
open html/index.html
```

### Options importantes du `Doxyfile`

Pour afficher les méthodes privées, variables membres, constantes, macros et fonctions statiques, le `Doxyfile` doit contenir au minimum :

```ini
INPUT                  = README.md README.user.md README.protocole_table.md src include
USE_MDFILE_AS_MAINPAGE = README.md
RECURSIVE              = NO
FILE_PATTERNS          = *.cpp *.h *.md

GENERATE_HTML          = YES
GENERATE_LATEX         = NO

EXTRACT_ALL            = YES
EXTRACT_PRIVATE        = YES
EXTRACT_STATIC         = YES
EXTRACT_LOCAL_CLASSES  = YES
EXTRACT_LOCAL_METHODS  = YES

HIDE_UNDOC_MEMBERS     = NO
HIDE_UNDOC_CLASSES     = NO

ENABLE_PREPROCESSING   = YES
MACRO_EXPANSION        = YES
EXPAND_ONLY_PREDEF     = NO

SOURCE_BROWSER         = YES
INLINE_SOURCES         = NO
REFERENCED_BY_RELATION = YES
REFERENCES_RELATION    = YES
GENERATE_TREEVIEW      = YES
```

### Où trouver les descriptions

Dans l’interface HTML Doxygen :

- le `README.md` est la page d’accueil
- les classes principales sont dans `Classes`
- les fonctions globales sont dans `Files`
- les constantes et macros sont visibles dans les pages de fichiers si `MACRO_EXPANSION` et `EXTRACT_ALL` sont actifs
- les membres privés sont visibles si `EXTRACT_PRIVATE = YES`
- les variables statiques et fonctions statiques sont visibles si `EXTRACT_STATIC = YES`

Les variables locales déclarées à l’intérieur d’une fonction ne sont pas documentées comme API par Doxygen. Pour les inspecter, utiliser l’onglet source généré par :

```ini
SOURCE_BROWSER = YES
```

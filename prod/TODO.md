# TODO - BLE Mesh Gateway ESPHome

## Étape 0 : Validation du code actuel
- [x] Compiler `prod/esp32-c6.yaml` sans erreur
- [x] Flash OTA sur l'ESP32-C6 (192.168.1.212)
- [x] Vérifier les logs de boot (mesh init, AppKey bound)
- [x] Re-provisionner le noeud ESP dans le réseau mesh (0x000C)

## Étape 1 : Valider ON/OFF
- [x] Tester ON/OFF Lilou (0x0005) — NO ACK ✅
- [x] Tester ON/OFF Julie (0x0004) — ACK ✅
- [x] Ajouter support ACK/NO ACK configurable par lampe

## Étape 2 : Luminosité
- [x] Sweep lightness Lilou NO ACK — range 1-65535 ✅
- [x] Sweep lightness Lilou ACK ✅
- [x] Sweep lightness Julie ACK ✅
- [x] Mettre à jour max_level de 50 à 65535 ✅
- [x] Valider luminosité Lilou via entité HA ✅
- [x] Valider luminosité Julie via entité HA ✅

## Étape 3 : CTL (température de couleur)
- [x] Ajouter `CONFIG_BLE_MESH_LIGHT_CTL_CLI` au sdkconfig ✅
- [x] Ajouter CTL client model au bridge C ✅
- [x] Implémenter `ble_mesh_bridge_send_ctl()` ✅
- [x] Sweep CTL Lilou 800-20000K ✅
- [x] Sweep CTL Julie 800-20000K ✅
- [x] Migrer Lilou vers `color_temperature` ✅
- [x] Migrer Julie vers `color_temperature` (ACK) ✅
- [x] Luminosité = Lightness SET + CTL ✅
- [x] Température = CTL SET, full range mesh 800-20000 ✅
- [x] Valider Julie luminosité + température dans HA ✅

## Étape 4 : RANGE_GET
- [x] ~~Lightness RANGE_GET~~ — timeout (non supporté par LEDVANCE)
- [x] ~~CTL Temperature RANGE_GET~~ — timeout (non supporté par LEDVANCE)
- [x] Alternative : ranges configurés manuellement ✅

## ✅ PROJET COMPLET

## Notes
- Julie (0x0004) : ACK ONLY
- Lilou (0x0005) : NO ACK (défaut)
- ESP-BLE-MESH node : 0x000C
- Plage lightness : 1-65535 (16 bits)
- Plage CTL mesh : 800-20000 (mapping interne)
- HA affichage : 2000K (chaud) → 6500K (froid)
- RANGE_GET non supporté par les lampes LEDVANCE (timeout systématique)
- ESPHome : 2026.3.3 / ESP-IDF 5.5.3

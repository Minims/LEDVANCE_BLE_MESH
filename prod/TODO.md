# TODO - BLE Mesh Gateway ESPHome

## Étape 0 : Validation du code actuel
- [x] Compiler `prod/esp32-c6.yaml` sans erreur
- [x] Flash OTA sur l'ESP32-C6 (192.168.1.212)
- [x] Vérifier les logs de boot (mesh init, AppKey bound)
- [x] Re-provisionner le noeud ESP dans le réseau mesh (0x000C)

## Étape 1 : Valider ON/OFF
- [x] Tester ON/OFF Lilou (0x0005) — NO ACK ✅
- [x] Tester ON/OFF Julie (0x0004) — ACK ✅ (OnOff ACK fonctionne)
- [x] Ajouter support ACK/NO ACK configurable par lampe (NO ACK par défaut)

## Étape 2 : Trouver la plage de luminosité
- [x] Sweep lightness Lilou NO ACK — range 1-65535 confirmé, changement graduel
- [x] Sweep lightness Lilou ACK — fonctionne aussi (ACK timeout mais commande reçue)
- [x] Mettre à jour max_level de 50 à 65535
- [x] Valider luminosité via entité lumière ESPHome (50% et 100% OK)
- [ ] Tester luminosité Julie (ACK, range 1-65535 à vérifier)
- [x] RANGE_GET ne fonctionne pas (timeout sur les deux lampes)

## Étape 3 : Ajouter CTL (température de couleur)
- [ ] Ajouter `CONFIG_BLE_MESH_LIGHT_CTL_CLI` au sdkconfig
- [ ] Ajouter CTL client model au bridge C
- [ ] Implémenter `ble_mesh_bridge_send_ctl()` (configurable ACK/NO ACK par lampe)
- [ ] Exposer `control_light_ctl()` dans le gateway C++
- [ ] Mettre à jour la YAML : passer de `monochromatic` à `color_temperature`

## Étape 4 : RANGE_GET automatique
- [x] ~~Implémenter send_lightness_range_get~~ — implémenté mais timeout sur les lampes
- [ ] Implémenter `ble_mesh_bridge_send_ctl_temperature_range_get()`
- [ ] Alternative : configurer min/max manuellement si les lampes ne supportent pas RANGE_GET

## Notes
- Julie (0x0004) : ACK ONLY — ne fonctionne pas en NO ACK, doit être configurée en ACK
- Lilou (0x0005) : ACK + NO ACK — fonctionne en NO ACK (défaut)
- ESP-BLE-MESH node : 0x000C
- Plage lightness : 1-65535 (16 bits complets)
- Gros changement visible autour de 5000-10000 (courbe non linéaire typique des LEDs)
- NO ACK par défaut pour toutes les lampes, configurable individuellement en ACK
- ACK Lightness timeout systématiquement (les lampes ne renvoient pas de réponse Lightness)
- RANGE_GET timeout sur les deux lampes (non supporté par les lampes LEDVANCE ?)

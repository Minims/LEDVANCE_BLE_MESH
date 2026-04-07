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
- [x] Sweep lightness Lilou NO ACK — range 1-65535, changement graduel ✅
- [x] Sweep lightness Lilou ACK — fonctionne (ACK timeout mais commande reçue) ✅
- [x] Mettre à jour max_level de 50 à 65535 ✅
- [x] Valider luminosité Lilou via entité lumière ESPHome ✅
- [ ] Tester luminosité Julie (ACK, range à vérifier)

## Étape 3 : CTL (température de couleur)
- [x] Ajouter `CONFIG_BLE_MESH_LIGHT_CTL_CLI` au sdkconfig ✅
- [x] Ajouter CTL client model au bridge C ✅ (auto-bindé AppKey)
- [x] Implémenter `ble_mesh_bridge_send_ctl()` ✅
- [x] Sweep CTL Lilou 800-20000K — changement graduel confirmé ✅
- [x] Migrer Lilou de `monochromatic` à `color_temperature` ✅
- [x] Luminosité via Lightness SET + CTL pour maintenir la température ✅
- [x] Température via CTL SET, full range mesh 800-20000 ✅
- [x] HA affiche 2000K-6500K ✅
- [ ] Tester CTL Julie (ACK, température à vérifier)
- [ ] Migrer Julie vers `color_temperature` si CTL fonctionne

## Étape 4 : RANGE_GET
- [x] ~~Lightness RANGE_GET~~ — timeout sur les deux lampes (non supporté)
- [ ] ~~CTL Temperature RANGE_GET~~ — probablement même résultat
- [x] Alternative : ranges configurés manuellement ✅

## Ce qui reste (Julie)
- [ ] Vérifier luminosité Julie via `set_mesh_lightness` service (ACK)
- [ ] Vérifier CTL Julie via `set_mesh_ctl` service (ACK)
- [ ] Migrer Julie vers `color_temperature` avec ACK
- [ ] Commit + push

## Notes
- Julie (0x0004) : ACK ONLY — doit être configurée en ACK
- Lilou (0x0005) : NO ACK (défaut), ACK aussi supporté
- ESP-BLE-MESH node : 0x000C
- Plage lightness : 1-65535 (16 bits)
- Plage CTL mesh : 800-20000 (interne, pas du vrai Kelvin)
- LEDVANCE CTL : gros changement visible sur tout le range 800-20000
- Luminosité : gros changement visible autour de 5000-10000
- ESPHome : 2026.3.3 / ESP-IDF 5.5.3

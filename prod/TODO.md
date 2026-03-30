# TODO - BLE Mesh Gateway ESPHome

## Étape 0 : Validation du code actuel
- [x] Compiler `prod/esp32-c6.yaml` sans erreur
- [x] Flash OTA sur l'ESP32-C6 (192.168.1.212)
- [x] Vérifier les logs de boot (mesh init, AppKey bound)
- [x] Re-provisionner le noeud ESP dans le réseau mesh (0x000C)

## Étape 1 : Valider ON/OFF
- [x] Tester ON/OFF Lilou (0x0005) — NO ACK → ✅ fonctionne
- [ ] Tester ON/OFF Julie (0x0004) — NO ACK ne fonctionne pas, tester avec ACK
- [ ] Ajouter support ACK/NO ACK configurable par lampe (NO ACK par défaut)
- [ ] Confirmer que les deux lampes répondent correctement

## Étape 2 : Trouver la plage de luminosité
- [ ] Tester différentes valeurs de lightness (1-255, 1-10000, 1-65535)
- [ ] Déterminer la plage réelle de fonctionnement de chaque lampe
- [ ] Implémenter RANGE_GET pour lire automatiquement min/max
- [ ] Gérer l'inversion de plage si nécessaire (luminosité ET température)

## Étape 3 : Ajouter CTL (température de couleur)
- [ ] Ajouter `CONFIG_BLE_MESH_LIGHT_CTL_CLI` au sdkconfig
- [ ] Ajouter CTL client model au bridge C
- [ ] Implémenter `ble_mesh_bridge_send_ctl()` (configurable ACK/NO ACK par lampe)
- [ ] Exposer `control_light_ctl()` dans le gateway C++
- [ ] Mettre à jour la YAML : passer de `monochromatic` à `color_temperature`

## Étape 4 : RANGE_GET automatique
- [ ] Implémenter `ble_mesh_bridge_send_lightness_range_get()`
- [ ] Implémenter `ble_mesh_bridge_send_ctl_temperature_range_get()`
- [ ] Parser les réponses RANGE_STATUS dans les callbacks
- [ ] Stocker les ranges par lampe dans le gateway
- [ ] Auto-envoyer les RANGE_GET quelques secondes après le boot

## Notes
- Julie (0x0004) : ACK ONLY — ne fonctionne pas en NO ACK, doit être configurée en ACK
- Lilou (0x0005) : ACK + NO ACK — fonctionne en NO ACK (défaut)
- ESP-BLE-MESH node : 0x000C
- Inversion de plage possible sur luminosité ET température (1→100 => 100→1)
- NO ACK par défaut pour toutes les lampes, configurable individuellement en ACK

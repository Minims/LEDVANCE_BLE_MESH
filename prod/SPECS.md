- Ceci est un composant esphome pour lampes BLE sigmesh.
- Compile pour mon esp32-C6 avec la yaml dans @prod/esp32-c6.yaml
- Corrige jusqu'a ce que ca compile sans erreur.
- Pour esphome install le via pip en venv dans ce repo. Utilise python 3.12 via pyenv and install ESPHome 2026.3.1
- ignore les dossiers @main et @web
- J'ai 2 lampes
  Lilou, addresse 0x005 : Gere ON/OFF, luminosité, temperature. Fonctionne en mode ACK et NO ACK
  Julie, addresse 0x004 : Gere ON/OFF, luminosité, temperature. Fonctionne en mode ACK ONLY
- Pour ameliorer la gestion des lampes, utilise les commandes SIGMESH pour lire le rabge sur la luminosité et la temperature histoire de revoir la plage de fonctionnement des lampes pour ces actions.
- Attention il peut etre necessaire d'inverser la plage de fonctionnement des lampes.(1->100) => (100->1)
- fait des commit bien séparé avec context genre fix(mesh): fix init
- tu as accès a esphome, analyse les logs, fait les actions via l'api esphome.
- Tu as des infos sur le reseau mesh dans @prod/nRF Mesh Network.json

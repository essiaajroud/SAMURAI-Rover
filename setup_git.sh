#!/bin/bash

# Initialiser le dépôt
git init

# Configurer Git (à adapter avec vos informations)
git config user.name "Your Name"
git config user.email "your.email@example.com"

# Créer la branche main
git checkout -b main

# Ajouter tous les fichiers
git add .

# Premier commit
git commit -m "Initial commit for SAMURAI-Rover"

# Ajouter le remote
git remote add origin https://github.com/essiaajroud/SAMURAI-Rover.git

# Pousser vers GitHub
git push -u origin main

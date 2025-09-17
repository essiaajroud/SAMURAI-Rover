#!/bin/bash

# Supprimer l'ancien .git s'il existe
rm -rf .git

# Initialiser un nouveau dépôt
git init

# Ajouter le remote
git remote add origin https://github.com/essiaajroud/SAMURAI-Rover.git

# Créer et basculer vers la branche main
git checkout -b main

# Ajouter tous les fichiers
git add .

# Premier commit
git commit -m "Initial commit"

# Forcer la mise à jour de la branche main distante
git push -f origin main

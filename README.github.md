# Procédure GitHub en ligne de commande

Ce document explique comment configurer Git, créer une clé SSH pour GitHub, ajouter cette clé dans GitHub, configurer le fichier `~/.ssh/config`, créer un dépôt GitHub depuis la ligne de commande avec `gh`, puis pousser un projet local vers GitHub.

Exemple utilisé dans ce document :

```text
Compte GitHub : jacqueslagnel
Dépôt GitHub  : lora_watering_slave
URL SSH       : git@github.com:jacqueslagnel/lora_watering_slave.git
```

---

## 1. Installer les outils nécessaires

### Installer Git

Sur Debian/Ubuntu :

```bash
sudo apt update
sudo apt install git
```

Vérifier l'installation :

```bash
git --version
```

### Installer GitHub CLI

GitHub CLI permet de créer un dépôt GitHub directement depuis le terminal.

```bash
sudo apt install gh
```

Vérifier l'installation :

```bash
gh --version
```

---

## 2. Configurer son identité Git

Configurer le nom utilisé dans les commits :

```bash
git config --global user.name "Jacques Lagnel"
```

Configurer l'adresse email utilisée dans les commits :

```bash
git config --global user.email "votre.email@example.com"
```

Vérifier la configuration :

```bash
git config --global --list
```

---

## 3. Créer une clé SSH pour GitHub

Créer une nouvelle clé SSH de type `ed25519` :

```bash
ssh-keygen -t ed25519 -C "votre.email@example.com"
```

Quand le terminal demande le nom du fichier, indiquer par exemple :

```text
~/.ssh/github_ed25519
```

Vous pouvez mettre une passphrase, ou laisser vide.

Après création, deux fichiers sont générés :

```text
~/.ssh/github_ed25519
~/.ssh/github_ed25519.pub
```

Important :

```text
github_ed25519      = clé privée, à ne jamais partager
github_ed25519.pub  = clé publique, à ajouter dans GitHub
```

---

## 4. Démarrer l'agent SSH et ajouter la clé

Démarrer l'agent SSH :

```bash
eval "$(ssh-agent -s)"
```

Ajouter la clé privée :

```bash
ssh-add ~/.ssh/github_ed25519
```

Vérifier que la clé est chargée :

```bash
ssh-add -l
```

---

## 5. Copier la clé publique

Afficher la clé publique :

```bash
cat ~/.ssh/github_ed25519.pub
```

Copier toute la ligne affichée.

Elle commence généralement par :

```text
ssh-ed25519
```

Ne jamais copier ni partager le fichier sans extension `.pub`.

---

## 6. Ajouter la clé SSH dans GitHub

Dans GitHub, aller dans :

```text
Settings → SSH and GPG keys → New SSH key
```

Remplir les champs :

```text
Title: Mon ordinateur
Key type: Authentication Key
Key: coller ici la clé publique
```

Puis cliquer sur :

```text
Add SSH key
```

---

## 7. Configurer le fichier SSH

Créer ou modifier le fichier :

```bash
nano ~/.ssh/config
```

Ajouter :

```sshconfig
Host github.com
    HostName github.com
    User git
    IdentityFile ~/.ssh/github_ed25519
    IdentitiesOnly yes
```

Corriger les permissions :

```bash
chmod 700 ~/.ssh
chmod 600 ~/.ssh/config
chmod 600 ~/.ssh/github_ed25519
chmod 644 ~/.ssh/github_ed25519.pub
```

---

## 8. Tester la connexion SSH à GitHub

Tester la connexion :

```bash
ssh -T git@github.com
```

La première fois, répondre :

```text
yes
```

Si tout fonctionne, GitHub affiche un message du type :

```text
Hi jacqueslagnel! You've successfully authenticated, but GitHub does not provide shell access.
```

Si le nom affiché n'est pas le bon compte GitHub, cela signifie probablement que la clé SSH est associée à un autre compte GitHub.

Pour voir quelle clé est utilisée :

```bash
ssh -vT git@github.com
```

Chercher une ligne du type :

```text
Offering public key: /home/votre_user/.ssh/github_ed25519
```

---

## 9. Se connecter avec GitHub CLI

Lancer :

```bash
gh auth login
```

Choisir généralement :

```text
GitHub.com
SSH
Login with a web browser
```

Vérifier l'authentification :

```bash
gh auth status
```

---

## 10. Initialiser un projet Git local

Se placer dans le dossier du projet :

```bash
cd lora_watering_slave
```

Initialiser Git :

```bash
git init
```

Ajouter les fichiers :

```bash
git add .
```

Créer le premier commit :

```bash
git commit -m "Initial commit"
```

Renommer la branche principale en `main` :

```bash
git branch -M main
```

---

## 11. Créer le dépôt GitHub depuis la ligne de commande

Avec `git` seul, il n'est pas possible de créer un dépôt directement sur GitHub.

Il faut utiliser GitHub CLI avec la commande `gh`.

Créer un dépôt privé :

```bash
gh repo create jacqueslagnel/lora_watering_slave --private
```

Créer un dépôt public :

```bash
gh repo create jacqueslagnel/lora_watering_slave --public
```

---

## 12. Ajouter le remote GitHub

Ajouter l'adresse SSH du dépôt distant :

```bash
git remote add origin git@github.com:jacqueslagnel/lora_watering_slave.git
```

Vérifier le remote :

```bash
git remote -v
```

Résultat attendu :

```text
origin  git@github.com:jacqueslagnel/lora_watering_slave.git (fetch)
origin  git@github.com:jacqueslagnel/lora_watering_slave.git (push)
```

---

## 13. Envoyer le projet vers GitHub

Pousser la branche `main` vers GitHub :

```bash
git push -u origin main
```

L'option `-u` associe la branche locale `main` à la branche distante `origin/main`.

Ensuite, les prochains envois pourront se faire simplement avec :

```bash
git push
```

---

## 14. Méthode rapide recommandée

Depuis le dossier du projet, après le commit initial :

```bash
gh repo create jacqueslagnel/lora_watering_slave --private --source=. --remote=origin --push
```

Pour un dépôt public :

```bash
gh repo create jacqueslagnel/lora_watering_slave --public --source=. --remote=origin --push
```

Cette commande fait automatiquement :

```text
création du dépôt GitHub
ajout du remote origin
push du projet vers GitHub
```

---

## 15. Erreur fréquente : Repository not found

Si la commande suivante échoue :

```bash
git push -u origin main
```

avec l'erreur :

```text
ERROR: Repository not found.
fatal: Could not read from remote repository.
```

Les causes possibles sont :

```text
le dépôt GitHub n'existe pas encore
le remote origin est incorrect
la clé SSH n'a pas accès au dépôt
vous êtes connecté au mauvais compte GitHub
```

Vérifier le remote :

```bash
git remote -v
```

Vérifier la connexion SSH :

```bash
ssh -T git@github.com
```

Vérifier que le dépôt existe :

```bash
gh repo view jacqueslagnel/lora_watering_slave
```

Si le dépôt n'existe pas, le créer :

```bash
gh repo create jacqueslagnel/lora_watering_slave --private
```

Puis pousser :

```bash
git push -u origin main
```

---

## 16. Modifier un remote existant

Si le remote `origin` est incorrect :

```bash
git remote set-url origin git@github.com:jacqueslagnel/lora_watering_slave.git
```

Si vous voulez supprimer puis recréer le remote :

```bash
git remote remove origin
git remote add origin git@github.com:jacqueslagnel/lora_watering_slave.git
```

---

## 17. Commandes utiles au quotidien

Voir l'état du projet :

```bash
git status
```

Ajouter les fichiers modifiés :

```bash
git add .
```

Créer un commit :

```bash
git commit -m "Description des modifications"
```

Envoyer vers GitHub :

```bash
git push
```

Récupérer les modifications depuis GitHub :

```bash
git pull
```

Voir l'historique :

```bash
git log --oneline
```

---

## 18. Résumé rapide

Pour créer un nouveau dépôt GitHub privé et pousser le projet :

```bash
git init
git add .
git commit -m "Initial commit"
git branch -M main
gh repo create jacqueslagnel/lora_watering_slave --private --source=. --remote=origin --push
```

Pour créer un dépôt public :

```bash
git init
git add .
git commit -m "Initial commit"
git branch -M main
gh repo create jacqueslagnel/lora_watering_slave --public --source=. --remote=origin --push
```

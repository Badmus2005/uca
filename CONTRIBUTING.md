# Contributing to Ucaotech DOFbot TRC2025

Merci de votre intérêt pour contribuer au projet Ucaotech DOFbot TRC2025 ! 🎉

## 📋 Table des Matières

1. [Code de Conduite](#code-de-conduite)
2. [Comment Contribuer](#comment-contribuer)
3. [Structure du Projet](#structure-du-projet)
4. [Standards de Code](#standards-de-code)
5. [Tests](#tests)
6. [Documentation](#documentation)
7. [Pull Requests](#pull-requests)

---

## 🤝 Code de Conduite

Ce projet adhère au [Contributor Covenant](https://www.contributor-covenant.org/). En participant, vous vous engagez à respecter ce code.

**Principes** :
- Respectueux et inclusif
- Constructif dans les critiques
- Focus sur ce qui est meilleur pour la communauté
- Empathique envers les autres membres

---

## 🚀 Comment Contribuer

### Types de Contributions

Nous acceptons plusieurs types de contributions :

1. **🐛 Rapporter des bugs** : Ouvrez une issue avec le tag `bug`
2. **💡 Proposer des features** : Ouvrez une issue avec le tag `enhancement`
3. **📝 Améliorer la documentation** : PRs sur `/docs`
4. **🔧 Corriger des bugs** : Soumettez un PR
5. **✨ Ajouter des features** : Discutez d'abord dans une issue

### Processus de Contribution

1. **Fork** le repository
2. **Clone** votre fork : `git clone https://github.com/votre-username/uca.git`
3. **Créer une branche** : `git checkout -b feature/ma-feature`
4. **Développer** votre contribution
5. **Tester** vos changements
6. **Commit** : `git commit -m "feat: Description de ma feature"`
7. **Push** : `git push origin feature/ma-feature`
8. **Ouvrir un Pull Request** sur GitHub

---

## 📂 Structure du Projet

Familiarisez-vous avec la structure :

```
ucaotech_dofbot_trc2025/
├── docs/              # Documentation complète
│   ├── guides/       # Guides utilisateur
│   ├── technical/    # Docs techniques
│   └── references/   # Références matériel
├── scripts/          # Scripts ROS et Python
│   ├── nodes/       # Nœuds ROS
│   └── utils/       # Utilitaires
├── trc2025_train_models/  # Entraînement ML
├── web/              # Interface web calibration
├── tests/            # Tests unitaires/intégration
└── config/           # Configurations
```

**Documentation importante** :
- Architecture : `docs/technical/ARCHITECTURE.md`
- API : `docs/technical/API_REFERENCE.md`
- Tests : `docs/technical/TESTING.md`

---

## 💻 Standards de Code

### Python

**Style** : [PEP 8](https://pep8.org/)

```python
# ✅ Bon
def calculate_trajectory(start_pos, end_pos, duration):
    """
    Calcule la trajectoire entre deux positions.
    
    Args:
        start_pos (list): Position de départ [x, y, z]
        end_pos (list): Position d'arrivée [x, y, z]
        duration (float): Durée du mouvement en secondes
        
    Returns:
        list: Points de trajectoire
    """
    pass

# ❌ Mauvais
def calcTraj(s,e,d):
    pass
```

**Conventions** :
- Noms de variables : `snake_case`
- Noms de classes : `PascalCase`
- Constantes : `UPPER_CASE`
- Fonctions privées : `_prefixe_underscore`

**Docstrings** : Style Google
```python
def ma_fonction(param1, param2):
    """
    Description courte.
    
    Description longue si nécessaire.
    
    Args:
        param1 (type): Description
        param2 (type): Description
        
    Returns:
        type: Description
        
    Raises:
        ExceptionType: Quand ça arrive
    """
    pass
```

### ROS Nodes

**Conventions ROS** :
- Noms de topics : `/lowercase_with_slashes`
- Noms de nodes : `node_name_lowercase`
- Rate : 10-50 Hz pour contrôle, 1-5 Hz pour monitoring

```python
#!/usr/bin/env python3
import rospy

class MonNode:
    def __init__(self):
        rospy.init_node('mon_node', anonymous=True)
        self.pub = rospy.Publisher('/mon_topic', MsgType, queue_size=10)
        self.sub = rospy.Subscriber('/autre_topic', MsgType, self.callback)
        
    def callback(self, msg):
        """Callback pour messages."""
        pass
        
    def run(self):
        """Boucle principale."""
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            # Logique
            rate.sleep()

if __name__ == '__main__':
    node = MonNode()
    node.run()
```

### Commits

**Format** : [Conventional Commits](https://www.conventionalcommits.org/)

```bash
# Format
<type>(<scope>): <description courte>

<description longue optionnelle>

# Types
feat:     Nouvelle feature
fix:      Correction bug
docs:     Documentation uniquement
style:    Formatage (pas de changement code)
refactor: Refactoring (ni fix ni feature)
test:     Ajout/modification tests
chore:    Maintenance (build, deps, etc.)

# Exemples
feat(vision): ajout détection multi-objets
fix(control): correction calcul cinématique inverse
docs(api): mise à jour référence API
test(integration): ajout tests vision-control
```

---

## 🧪 Tests

### Exécuter les Tests

```bash
# Tous les tests
pytest tests/

# Tests unitaires seulement
pytest tests/unit/

# Tests avec coverage
pytest --cov=scripts --cov=src tests/

# Tests spécifiques
pytest tests/unit/test_vision_node.py -v
```

### Écrire des Tests

**Structure** :
```python
# tests/unit/test_mon_module.py
import pytest
from scripts.mon_module import MaClasse

class TestMaClasse:
    """Tests pour MaClasse."""
    
    @pytest.fixture
    def instance(self):
        """Fixture pour créer instance."""
        return MaClasse()
    
    def test_methode_basique(self, instance):
        """Test méthode basique."""
        result = instance.ma_methode(param=42)
        assert result == expected_value
        
    def test_methode_avec_erreur(self, instance):
        """Test gestion erreur."""
        with pytest.raises(ValueError):
            instance.ma_methode(param=-1)
```

**Bonnes pratiques** :
- 1 test = 1 concept
- Noms descriptifs : `test_fonction_condition_resultat`
- Arrange, Act, Assert (AAA)
- Mocks pour dépendances externes
- Coverage minimum : 80%

---

## 📚 Documentation

### Documenter le Code

**Tous les modules** doivent avoir un docstring :
```python
"""
Module pour le contrôle du bras robotique.

Ce module fournit des classes pour contrôler le DOFbot via I2C,
incluant la cinématique inverse et la planification de trajectoire.

Exemple:
    >>> from control import ArmController
    >>> arm = ArmController()
    >>> arm.move_to([0.3, 0.0, 0.2])
"""
```

### Mettre à Jour la Documentation

Si votre PR modifie :
- **API** : Mettre à jour `docs/technical/API_REFERENCE.md`
- **Architecture** : Mettre à jour `docs/technical/ARCHITECTURE.md`
- **Usage** : Mettre à jour guides dans `docs/guides/`
- **Config** : Documenter dans fichiers YAML

### Style Documentation

- Markdown pour toute documentation
- Code blocks avec langage : ` ```python `
- Screenshots dans `docs/images/`
- Diagrammes en ASCII art ou Mermaid

---

## 🔀 Pull Requests

### Checklist PR

Avant de soumettre votre PR, vérifiez :

- [ ] Code suit les standards (PEP 8)
- [ ] Docstrings ajoutés/mis à jour
- [ ] Tests ajoutés pour nouvelle feature
- [ ] Tests existants passent : `pytest tests/`
- [ ] Documentation mise à jour
- [ ] Commits formatés (Conventional Commits)
- [ ] Branch à jour avec `main`
- [ ] Pas de conflits

### Template PR

```markdown
## Description
Brève description des changements.

## Type de changement
- [ ] Bug fix
- [ ] Nouvelle feature
- [ ] Breaking change
- [ ] Documentation

## Tests
Décrivez les tests effectués :
- [ ] Tests unitaires
- [ ] Tests d'intégration
- [ ] Tests manuels

## Checklist
- [ ] Code suit les standards
- [ ] Documentation mise à jour
- [ ] Tests passent
```

### Revue de Code

Votre PR sera revu par les mainteneurs. Soyez patient et réceptif aux feedbacks !

**Critères de revue** :
- Qualité du code
- Tests adéquats
- Documentation claire
- Pas de régression
- Compatibilité

---

## 🐛 Rapporter des Bugs

### Template Issue Bug

```markdown
**Describe the bug**
Description claire et concise du bug.

**To Reproduce**
Steps to reproduce:
1. Go to '...'
2. Click on '...'
3. See error

**Expected behavior**
Ce qui devrait se passer.

**Screenshots**
Si applicable, ajoutez des screenshots.

**Environment:**
 - OS: [e.g. Ubuntu 20.04]
 - Python: [e.g. 3.8.10]
 - ROS: [e.g. Noetic]
 - Version: [e.g. v1.0.0]

**Additional context**
Contexte additionnel.
```

---

## 💡 Proposer des Features

### Template Issue Feature

```markdown
**Is your feature request related to a problem?**
Description claire du problème.

**Describe the solution you'd like**
Description de la solution souhaitée.

**Describe alternatives you've considered**
Alternatives considérées.

**Additional context**
Contexte additionnel, screenshots, etc.
```

---

## 📞 Contact

**Équipe Ucaotech**
- GitHub : [@Badmus2005](https://github.com/Badmus2005)
- Email : ucaotech@ensam.ac.ma

**Resources**
- Documentation : `docs/INDEX.md`
- Issues : [GitHub Issues](https://github.com/Badmus2005/uca/issues)
- Discussions : [GitHub Discussions](https://github.com/Badmus2005/uca/discussions)

---

## 🙏 Remerciements

Merci à tous les contributeurs qui aident à améliorer ce projet !

**Contributors** :
- [Liste sera générée automatiquement]

---

**Bonne contribution ! 🚀**

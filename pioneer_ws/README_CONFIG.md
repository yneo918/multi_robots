# Configuration Management

This project uses a template-based configuration system to avoid merge conflicts.

## Setup Instructions

### First Time Setup

1. Run `./scripts/setup.sh`
2. The script will create `config/system_config.yaml` from the template
3. Edit `config/system_config.yaml` with your robot-specific settings:
   - `robot.id`: Your robot ID (p1, p2, p3, etc.)
   - `user.name`: Your username
   - `user.home_dir`: Your home directory
4. Run `./scripts/setup.sh` again to complete setup

### Configuration Files

- `config/system_config.yaml.default`: Template file (tracked in git)
- `config/system_config.yaml`: Your local configuration (ignored by git)

### Updating Configuration

When pulling updates that modify the template:
1. The setup script will warn you if the template is newer
2. Review changes with: `diff config/system_config.yaml config/system_config.yaml.default`
3. Manually apply any new settings you need

### Benefits

- No merge conflicts on `system_config.yaml`
- Each robot maintains its own configuration
- New features are added to the template only
- Local settings are preserved across updates
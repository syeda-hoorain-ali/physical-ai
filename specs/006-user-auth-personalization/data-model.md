# Data Model: User Authentication with Personalized Content

## Entities

### User Profile
**Description**: Represents a registered user with authentication credentials and background information

**Fields**:
- `id` (string): Unique identifier for the user
- `email` (string): User's email address (required, unique)
- `name` (string): User's display name
- `createdAt` (datetime): Account creation timestamp
- `updatedAt` (datetime): Last update timestamp
- `softwareExperience` (string): User's software development experience level
- `hardwareAccess` (string): Current hardware access (e.g., GPU type, robotics kit)
- `programmingLanguages` (string): Familiar programming languages
- `hardwareExperience` (string): Experience with hardware development
- `learningGoals` (string): User's learning objectives
- `educationalBackground` (string): Educational background and field
- `timeCommitment` (string): Available time for learning per week
- `aiMlExperience` (string): Experience with AI/ML concepts
- `rosExperience` (string): Experience with ROS/ROS2
- `simulationExperience` (string): Experience with simulation tools

**Validation Rules**:
- Email must be valid format
- Software experience must be one of: "beginner", "intermediate", "advanced"
- Hardware access must match available options from system
- Required fields: email, softwareExperience, hardwareAccess

### Session
**Description**: Represents an active user session

**Fields**:
- `id` (string): Unique session identifier
- `userId` (string): Reference to User Profile
- `expiresAt` (datetime): Session expiration time
- `createdAt` (datetime): Session creation time
- `lastAccessed` (datetime): Last time session was used

## Relationships
- User Profile (1) : Session (0..n) - One user can have multiple active sessions

## State Transitions
- User Profile: Created during registration → Updated when user modifies profile
- Session: Created during login → Refreshed on activity → Expired after timeout
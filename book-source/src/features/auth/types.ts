export interface UserProfile {
  id: string;
  user_id: string;
  email: string | null;
  full_name: string | null;
  programming_level: 'none' | 'beginner' | 'intermediate' | 'advanced' | 'expert';
  programming_languages: string[];
  ai_ml_experience: boolean;
  ros_experience: boolean;
  hardware_access: 'none' | 'basic' | 'moderate' | 'advanced';
  gpu_type: string | null;
  has_robotics_hardware: boolean;
  robotics_hardware_details: string | null;
  learning_goals: string[];
  onboarding_completed: boolean;
  created_at: string;
  updated_at: string;
}

export interface ProfileUpdate {
  full_name?: string;
  programming_level?: 'none' | 'beginner' | 'intermediate' | 'advanced' | 'expert';
  programming_languages?: string[];
  ai_ml_experience?: boolean;
  ros_experience?: boolean;
  hardware_access?: 'none' | 'basic' | 'moderate' | 'advanced';
  gpu_type?: string;
  has_robotics_hardware?: boolean;
  robotics_hardware_details?: string;
  learning_goals?: string[];
  onboarding_completed?: boolean;
}

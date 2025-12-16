import { useState, useEffect } from 'react';
import { Button } from '@site/src/components/ui/button';
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '@site/src/components/ui/card';
import { Form, FormControl, FormField, FormItem, FormLabel, FormMessage } from '@site/src/components/ui/form';
import { Input } from '@site/src/components/ui/input';
import { Label } from '@site/src/components/ui/label';
import { Checkbox } from '@site/src/components/ui/checkbox';
import { Textarea } from '@site/src/components/ui/textarea';
import { RadioGroup, RadioGroupItem } from '@site/src/components/ui/radio-group';
import { cn } from '@site/src/lib/utils';
import { useAuth, useProfile } from '@site/src/features/auth/hooks';
import { userProfileSchema, UserProfileFormData } from '@site/src/features/auth/schema';
import { useForm } from 'react-hook-form';
import { zodResolver } from '@hookform/resolvers/zod';
import { useHistory } from '@docusaurus/router';
import { BookOpen, Loader2, ArrowLeft, Save, LogOut, Code, Cpu, Target, User } from 'lucide-react';

const PROGRAMMING_LANGUAGES = [
  'Python', 'C++', 'C', 'JavaScript/TypeScript', 'Rust', 'Go', 'Java', 'MATLAB'
];

const LEARNING_GOALS = [
  'Build autonomous robots',
  'Master ROS 2 development',
  'Learn AI/ML for robotics',
  'Create simulations in Gazebo/Unity',
  'Work with NVIDIA Isaac',
  'Develop humanoid robots',
  'Research embodied AI',
  'Career transition to robotics'
];

export const UserProfile = () => {

  const history = useHistory();
  const { user, loading: authLoading, signOut } = useAuth();
  const { profile, loading: profileLoading, updateProfile } = useProfile();

  const [saving, setSaving] = useState(false);


  const form = useForm({
    resolver: zodResolver(userProfileSchema),
    defaultValues: {
      fullName: '',
      email: '',
      programmingLevel: 'beginner',
      programmingLanguages: [],
      aiMlExperience: false,
      rosExperience: false,
      hardwareAccess: 'none',
      gpuType: '',
      hasRoboticsHardware: false,
      roboticsHardwareDetails: '',
      learningGoals: [],
    },
    values: profile ? {
      fullName: profile.full_name || '',
      email: profile.email || '',
      programmingLevel: profile.programming_level || 'beginner',
      programmingLanguages: profile.programming_languages || [],
      aiMlExperience: profile.ai_ml_experience || false,
      rosExperience: profile.ros_experience || false,
      hardwareAccess: profile.hardware_access || 'none',
      gpuType: profile.gpu_type || '',
      hasRoboticsHardware: profile.has_robotics_hardware || false,
      roboticsHardwareDetails: profile.robotics_hardware_details || '',
      learningGoals: profile.learning_goals || [],
    } : undefined
  })

  useEffect(() => {
    if (!authLoading && !user) {
      history.push('/auth');
    }
  }, [user, authLoading]);



  const onSubmit = async (data: UserProfileFormData) => {
    setSaving(true);
    await updateProfile(data);
    setSaving(false);
  };

  const handleSignOut = async () => {
    await signOut();
    history.push('/');
  };

  if (authLoading || profileLoading) {
    return (
      <div className="flex min-h-screen items-center justify-center bg-background">
        <Loader2 className="h-8 w-8 animate-spin text-primary" />
      </div>
    );
  }

  return (
    <div className="min-h-screen bg-background py-8">
      <div className="container mx-auto max-w-3xl px-4">
        <div className="mb-8 flex items-center justify-between">
          <Button variant="ghost" onClick={() => history.push('/')}>
            <ArrowLeft className="mr-2 h-4 w-4" />
            Back to Home
          </Button>
          <Button variant="outline" onClick={handleSignOut}>
            <LogOut className="mr-2 h-4 w-4" />
            Sign Out
          </Button>
        </div>

        <div className="mb-8 text-center">
          <div className="mb-4 flex justify-center">
            <div className="rounded-xl bg-linear-to-br from-primary to-secondary p-3">
              <BookOpen className="h-8 w-8 text-primary-foreground" />
            </div>
          </div>
          <h1 className="text-2xl font-bold text-foreground">Your Profile</h1>
          <p className="mt-2 text-muted-foreground">
            Update your background to improve content personalization
          </p>
        </div>


        <Form {...form}>
          <form onSubmit={form.handleSubmit(onSubmit)} className="space-y-6">
            {/* Basic Info */}
            <Card className="border-border/50 bg-card/80 backdrop-blur">
              <CardHeader>
                <div className="flex items-center gap-2">
                  <User className="h-5 w-5 text-primary" />
                  <CardTitle>Basic Information</CardTitle>
                </div>
              </CardHeader>

              <CardContent className="space-y-4">
                <FormField
                  control={form.control}
                  name="fullName"
                  render={({ field }) => (
                    <FormItem className="space-y-2">
                      <FormLabel>Full Name</FormLabel>
                      <FormControl>
                        <Input placeholder="Your name" {...field} />
                      </FormControl>
                      <FormMessage />
                    </FormItem>
                  )}
                />

                <FormField
                  control={form.control}
                  name="email"
                  render={({ field }) => (
                    <FormItem className="space-y-2">
                      <FormLabel>Email</FormLabel>
                      <FormControl>
                        <Input type="email" disabled className="bg-muted" {...field} />
                      </FormControl>
                      <FormMessage />
                    </FormItem>
                  )}
                />
              </CardContent>
            </Card>

            {/* Software Background */}
            <Card className="border-border/50 bg-card/80 backdrop-blur">
              <CardHeader>
                <div className="flex items-center gap-2">
                  <Code className="h-5 w-5 text-primary" />
                  <CardTitle>Software Background</CardTitle>
                </div>
                <CardDescription>Your programming experience</CardDescription>
              </CardHeader>

              <CardContent className="space-y-6">
                <FormField
                  name="programmingLevel"
                  control={form.control}
                  render={({ field }) => (
                    <FormItem className="space-y-3">
                      <FormLabel>Programming Experience Level</FormLabel>
                      <FormControl>
                        <RadioGroup
                          {...field}
                          onValueChange={field.onChange}
                          className="grid grid-cols-2 gap-2 sm:grid-cols-3"
                        >
                          {['none', 'beginner', 'intermediate', 'advanced', 'expert'].map((level) => (
                            <Label
                              key={level}
                              className={cn(
                                "flex cursor-pointer items-center gap-2 rounded-lg border p-3 transition-colors",
                                field.value === level ? 'border-primary bg-primary/10' : 'border-border hover:bg-muted/50'
                              )}
                            >
                              <RadioGroupItem value={level} className="peer sr-only" />
                              <span className="capitalize">{level}</span>
                            </Label>
                          ))}
                        </RadioGroup>
                      </FormControl>
                      <FormMessage />
                    </FormItem>
                  )}
                />

                <FormField
                  name="programmingLanguages"
                  control={form.control}
                  render={({ field }) => (
                    <FormItem className="space-y-2">
                      <FormLabel>Languages You Know</FormLabel>
                      <FormControl>
                        <div className="grid grid-cols-2 gap-2 sm:grid-cols-4">
                          {PROGRAMMING_LANGUAGES.map((lang) => (
                            <Label
                              key={lang}
                              className={cn(
                                "flex cursor-pointer items-center gap-2 rounded-lg border p-3 transition-colors",
                                field.value.includes(lang) ? 'border-primary bg-primary/10' : 'border-border hover:bg-muted/50'
                              )}
                            >
                              <Checkbox
                                checked={field.value.includes(lang)}
                                onCheckedChange={(checked) => {
                                  return field.onChange(
                                    checked
                                      ? [...field.value, lang]
                                      : field.value.filter((value) => value !== lang)
                                  );
                                }}
                              />
                              <span className="text-sm">{lang}</span>
                            </Label>
                          ))}
                        </div>
                      </FormControl>
                      <FormMessage />
                    </FormItem>
                  )}
                />

                <FormField
                  name="aiMlExperience"
                  control={form.control}
                  render={({ field }) => (
                    <FormItem className="flex flex-col gap-3 sm:flex-row">
                      <FormControl>
                        <FormLabel
                          className={cn(
                            "flex flex-1 cursor-pointer items-center gap-2 rounded-lg border p-4 transition-colors",
                            field.value ? 'border-primary bg-primary/10' : 'border-border hover:bg-muted/50'
                          )}
                        >
                          <Checkbox
                            checked={field.value}
                            onCheckedChange={field.onChange}
                          />
                          <span>I have AI/ML experience</span>
                        </FormLabel>
                      </FormControl>
                      <FormMessage />
                    </FormItem>
                  )}
                />
                <FormField
                  name="rosExperience"
                  control={form.control}
                  render={({ field }) => (
                    <FormItem className="flex flex-col gap-3 sm:flex-row">
                      <FormControl>
                        <FormLabel
                          className={cn(
                            "flex flex-1 cursor-pointer items-center gap-2 rounded-lg border p-4 transition-colors",
                            field.value ? 'border-primary bg-primary/10' : 'border-border hover:bg-muted/50'
                          )}
                        >
                          <Checkbox
                            checked={field.value}
                            onCheckedChange={field.onChange}
                          />
                          <span>I have ROS/ROS 2 experience</span>
                        </FormLabel>
                      </FormControl>
                      <FormMessage />
                    </FormItem>
                  )}
                />
              </CardContent>
            </Card>

            {/* Hardware Access */}
            <Card className="border-border/50 bg-card/80 backdrop-blur">
              <CardHeader>
                <div className="flex items-center gap-2">
                  <Cpu className="h-5 w-5 text-secondary" />
                  <CardTitle>Hardware Access</CardTitle>
                </div>
                <CardDescription>Your hardware capabilities</CardDescription>
              </CardHeader>

              <CardContent className="space-y-6">
                <FormField
                  name="hardwareAccess"
                  control={form.control}
                  render={({ field }) => (
                    <FormItem className="space-y-3">
                      <FormLabel>GPU Computing Access</FormLabel>
                      <FormControl>
                        <RadioGroup
                          {...field}
                          onValueChange={field.onChange}
                          className="space-y-2"
                        >
                          {[
                            { value: 'none', label: 'No GPU', desc: "I'll use cloud resources or CPU-only" },
                            { value: 'basic', label: 'Basic GPU', desc: 'GTX 1060/1070 or similar' },
                            { value: 'moderate', label: 'Mid-range GPU', desc: 'RTX 3070/3080 or similar' },
                            { value: 'advanced', label: 'High-end GPU', desc: 'RTX 4080/4090 or datacenter GPUs' },
                          ].map((opt) => (
                            <Label
                              key={opt.value}
                              className={cn(
                                "flex cursor-pointer items-center gap-2 rounded-lg border p-4 transition-colors",
                                field.value === opt.value ? 'border-primary bg-primary/10' : 'border-border hover:bg-muted/50'
                              )}
                            >
                              <RadioGroupItem value={opt.value} className="peer sr-only" />
                              <div>
                                <span className="font-medium">{opt.label}</span>
                                <p className="text-sm text-muted-foreground">{opt.desc}</p>
                              </div>
                            </Label>
                          ))}
                        </RadioGroup>
                      </FormControl>
                      <FormMessage />
                    </FormItem>
                  )}
                />

                <FormField
                  name="gpuType"
                  control={form.control}
                  render={({ field }) => (
                    form.watch('hardwareAccess') !== 'none' && (
                      <FormItem className="space-y-2">
                        <FormLabel>GPU Model (optional)</FormLabel>
                        <FormControl>
                          <Textarea
                            placeholder="e.g., NVIDIA RTX 4090"
                            className="resize-none"
                            rows={2}
                            {...field}
                          />
                        </FormControl>
                        <FormMessage />
                      </FormItem>
                    )
                  )}
                />

                <FormField
                  name="hasRoboticsHardware"
                  control={form.control}
                  render={({ field }) => (
                    <FormItem className="space-y-3">
                      <FormControl>
                        <FormLabel
                          className={cn(
                            "flex cursor-pointer items-center gap-2 rounded-lg border p-4 transition-colors",
                            field.value ? 'border-primary bg-primary/10' : 'border-border hover:bg-muted/50'
                          )}
                        >
                          <Checkbox
                            checked={field.value}
                            onCheckedChange={field.onChange}
                          />
                          <div>
                            <span className="font-medium">I have robotics hardware</span>
                            <p className="text-sm text-muted-foreground">Robot kits, arms, drones, etc.</p>
                          </div>
                        </FormLabel>
                      </FormControl>
                      <FormMessage />
                    </FormItem>
                  )}
                />

                <FormField
                  name="roboticsHardwareDetails"
                  control={form.control}
                  render={({ field }) => (
                    form.watch('hasRoboticsHardware') && (
                      <FormItem className="space-y-2">
                        <FormLabel>Describe your robotics hardware</FormLabel>
                        <FormControl>
                          <Textarea
                            placeholder="e.g., TurtleBot 4, Unitree Go2..."
                            className="resize-none"
                            rows={3}
                            {...field}
                          />
                        </FormControl>
                        <FormMessage />
                      </FormItem>
                    )
                  )}
                />
              </CardContent>
            </Card>

            {/* Learning Goals */}
            <Card className="border-border/50 bg-card/80 backdrop-blur">
              <CardHeader>
                <div className="flex items-center gap-2">
                  <Target className="h-5 w-5 text-primary" />
                  <CardTitle>Learning Goals</CardTitle>
                </div>
                <CardDescription>What you want to achieve</CardDescription>
              </CardHeader>

              <CardContent>
                <FormField
                  name="learningGoals"
                  control={form.control}
                  render={({ field }) => (
                    <FormItem>
                      <div className="grid gap-2 sm:grid-cols-2">
                        {LEARNING_GOALS.map((goal) => (
                          <FormLabel
                            key={goal}
                            className={cn(
                              "flex cursor-pointer items-center gap-2 rounded-lg border p-4 transition-colors",
                              field.value.includes(goal) ? 'border-primary bg-primary/10' : 'border-border hover:bg-muted/50'
                            )}
                          >
                            <Checkbox
                              checked={field.value.includes(goal)}
                              onCheckedChange={(checked) => {
                                return field.onChange(
                                  checked
                                    ? [...field.value, goal]
                                    : field.value.filter((value) => value !== goal)
                                );
                              }}
                            />
                            <span className="text-sm">{goal}</span>
                          </FormLabel>
                        ))}
                      </div>
                      <FormMessage />
                    </FormItem>
                  )}
                />
              </CardContent>
            </Card>

            {/* Save Button */}
            <div className="flex justify-end">
              <Button type="submit" disabled={saving} size="lg">
                {saving ? <Loader2 className="mr-2 h-4 w-4 animate-spin" /> : <Save className="mr-2 h-4 w-4" />}
                Save Changes
              </Button>
            </div>
          </form>
        </Form>
      </div>
    </div>
  );
};


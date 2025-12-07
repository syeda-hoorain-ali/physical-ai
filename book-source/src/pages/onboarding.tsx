import { useState, useEffect } from 'react';
import { Button } from '@site/src/components/ui/button';
import { Card, CardContent, CardDescription, CardHeader, CardTitle } from '@site/src/components/ui/card';
import { Form, FormControl, FormField, FormItem, FormLabel, FormMessage } from '@site/src/components/ui/form';
import { Label } from '@site/src/components/ui/label';
import { Checkbox } from '@site/src/components/ui/checkbox';
import { Textarea } from '@site/src/components/ui/textarea';
import { RadioGroup, RadioGroupItem } from '@site/src/components/ui/radio-group';
import { cn } from '@site/src/lib/utils';
import { useAuth, useProfile } from '@site/src/features/auth/hooks';
import { useForm } from 'react-hook-form';
import { zodResolver } from '@hookform/resolvers/zod';
import { userProfileSchema, UserProfileFormData } from '@site/src/features/auth/schema';
import { useHistory } from '@docusaurus/router';
import { BookOpen, Loader2, ArrowRight, ArrowLeft, Code, Cpu, Target } from 'lucide-react';

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

const Onboarding = () => {

  const history = useHistory();
  const { user, loading: authLoading } = useAuth();
  const { profile, loading: profileLoading, updateProfile } = useProfile();

  const [step, setStep] = useState(1);
  const [saving, setSaving] = useState(false);

  const form = useForm({
    resolver: zodResolver(userProfileSchema),
    defaultValues: {
      programmingLevel: 'beginner',
      programmingLanguages: [],
      aiMlExperience: false,
      rosExperience: false,
      hardwareAccess: 'none',
      gpuType: '',
      hasRoboticsHardware: false,
      roboticsHardwareDetails: '',
      learningGoals: [],
    }
  });

  useEffect(() => {
    if (!authLoading && !user) {
      history.push('/auth');
    }
  }, [user, authLoading]);

  useEffect(() => {
    if (profile?.onboarding_completed) {
      history.push('/');
    }
  }, [profile]);

  const handleComplete = async (data: UserProfileFormData) => {
    setSaving(true);
    const { error } = await updateProfile(data);
    setSaving(false);

    if (!error) {
      history.push('/');
    }
  };

  if (authLoading || profileLoading) {
    return (
      <div className="flex min-h-screen items-center justify-center bg-background">
        <Loader2 className="h-8 w-8 animate-spin text-primary" />
      </div>
    );
  }

  return (
    <div className="flex min-h-screen items-center justify-center bg-background p-4">
      <div className="w-full max-w-2xl space-y-6">
        <div className="text-center">
          <div className="mb-4 flex justify-center">
            <div className="rounded-xl bg-linear-to-br from-primary to-secondary p-3">
              <BookOpen className="h-8 w-8 text-primary-foreground" />
            </div>
          </div>
          <h1 className="text-2xl font-bold text-foreground">Let's Personalize Your Experience</h1>
          <p className="mt-2 text-muted-foreground">
            Tell us about your background so we can customize your learning journey
          </p>
        </div>

        {/* Progress indicator */}
        <div className="flex items-center justify-center gap-2">
          {[1, 2, 3].map((s) => (
            <div
              key={s}
              className={`h-2 w-16 rounded-full transition-colors ${
                s === step ? 'bg-primary' : s < step ? 'bg-primary/50' : 'bg-muted'
              }`}
            />
          ))}
        </div>

        <Form {...form}>
          <form onSubmit={form.handleSubmit(handleComplete)}>
            <Card className="border-border/50 bg-card/80 backdrop-blur">
              {step === 1 && (
                <>
                  <CardHeader>
                    <div className="flex items-center gap-2">
                      <Code className="h-5 w-5 text-primary" />
                      <CardTitle>Software Background</CardTitle>
                    </div>
                    <CardDescription>
                      Help us understand your programming experience
                    </CardDescription>
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
                        <FormItem className="space-y-3">
                          <FormLabel>Languages You Know (select all that apply)</FormLabel>
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

                    <div className="flex flex-col gap-3 sm:flex-row">
                      <FormField
                        name="aiMlExperience"
                        control={form.control}
                        render={({ field }) => (
                          <FormItem className="flex-1">
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
                          <FormItem className="flex-1">
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
                                <span>I have ROS/ROS 2 experience</span>
                              </FormLabel>
                            </FormControl>
                            <FormMessage />
                          </FormItem>
                        )}
                      />
                    </div>
                  </CardContent>
                </>
              )}

              {step === 2 && (
                <>
                  <CardHeader>
                    <div className="flex items-center gap-2">
                      <Cpu className="h-5 w-5 text-secondary" />
                      <CardTitle>Hardware Access</CardTitle>
                    </div>
                    <CardDescription>
                      Tell us about your hardware capabilities for simulations
                    </CardDescription>
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
                              <Label
                                className={cn(
                                  "flex cursor-pointer items-center gap-2 rounded-lg border p-4 transition-colors",
                                  field.value === 'none' ? 'border-primary bg-primary/10' : 'border-border hover:bg-muted/50'
                                )}
                              >
                                <RadioGroupItem value="none" className="peer sr-only" />
                                <div>
                                  <span className="font-medium">No GPU</span>
                                  <p className="text-sm text-muted-foreground">I'll use cloud resources or CPU-only</p>
                                </div>
                              </Label>
                              <Label
                                className={cn(
                                  "flex cursor-pointer items-center gap-2 rounded-lg border p-4 transition-colors",
                                  field.value === 'basic' ? 'border-primary bg-primary/10' : 'border-border hover:bg-muted/50'
                                )}
                              >
                                <RadioGroupItem value="basic" className="peer sr-only" />
                                <div>
                                  <span className="font-medium">Basic GPU</span>
                                  <p className="text-sm text-muted-foreground">GTX 1060/1070 or similar</p>
                                </div>
                              </Label>
                              <Label
                                className={cn(
                                  "flex cursor-pointer items-center gap-2 rounded-lg border p-4 transition-colors",
                                  field.value === 'moderate' ? 'border-primary bg-primary/10' : 'border-border hover:bg-muted/50'
                                )}
                              >
                                <RadioGroupItem value="moderate" className="peer sr-only" />
                                <div>
                                  <span className="font-medium">Mid-range GPU</span>
                                  <p className="text-sm text-muted-foreground">RTX 3070/3080 or similar</p>
                                </div>
                              </Label>
                              <Label
                                className={cn(
                                  "flex cursor-pointer items-center gap-2 rounded-lg border p-4 transition-colors",
                                  field.value === 'advanced' ? 'border-primary bg-primary/10' : 'border-border hover:bg-muted/50'
                                )}
                              >
                                <RadioGroupItem value="advanced" className="peer sr-only" />
                                <div>
                                  <span className="font-medium">High-end GPU</span>
                                  <p className="text-sm text-muted-foreground">RTX 4080/4090 or datacenter GPUs</p>
                                </div>
                              </Label>
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
                                placeholder="e.g., NVIDIA RTX 4090, AMD RX 7900 XTX"
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
                                placeholder="e.g., TurtleBot 4, Unitree Go2, Custom 6DOF arm..."
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
                </>
              )}

              {step === 3 && (
                <>
                  <CardHeader>
                    <div className="flex items-center gap-2">
                      <Target className="h-5 w-5 text-primary" />
                      <CardTitle>Learning Goals</CardTitle>
                    </div>
                    <CardDescription>
                      What do you want to achieve with this textbook?
                    </CardDescription>
                  </CardHeader>
                  <CardContent className="space-y-6">
                    <FormField
                      name="learningGoals"
                      control={form.control}
                      render={({ field }) => (
                        <FormItem>
                          <FormLabel>Select your goals (choose all that apply)</FormLabel>
                          <FormControl>
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
                          </FormControl>
                          <FormMessage />
                        </FormItem>
                      )}
                    />
                  </CardContent>
                </>
              )}

              {/* Navigation */}
              <div className="flex justify-between border-t border-border/50 p-6">
                {step > 1 ? (
                  <Button
                    variant="outline"
                    onClick={(e) => {
                      e.preventDefault();
                      setStep(step - 1);
                    }}
                  >
                    <ArrowLeft className="mr-2 h-4 w-4" />
                    Back
                  </Button>
                ) : (
                  <div />
                )}

                {step < 3 ? (
                  <Button
                    onClick={(e) => {
                      e.preventDefault();
                      setStep(step + 1);
                    }}
                  >
                    Next
                    <ArrowRight className="ml-2 h-4 w-4" />
                  </Button>
                ) : (
                  <Button type="submit" disabled={saving}>
                    {saving ? <Loader2 className="mr-2 h-4 w-4 animate-spin" /> : null}
                    Complete Setup
                  </Button>
                )}
              </div>
            </Card>
          </form>
        </Form>

        <p className="text-center text-sm text-muted-foreground">
          You can update these settings anytime in your profile
        </p>
      </div>
    </div>
  );
};

export default Onboarding;
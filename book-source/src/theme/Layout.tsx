import React from 'react';
import OriginalLayout from '@theme-original/Layout';
import type { Props } from '@theme/Layout';
import { Toaster } from '../components/ui/sonner';

const Layout: React.FC<Props> = (props) => {
  return (<>
    <Toaster />
    <OriginalLayout {...props}>{props.children}</OriginalLayout>
  </>);
};

export default Layout;
